/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This class bridges serial IO between a master (upstream) and slave (downstream)

Notes:		The logic for rx/tx is always facing away from the middle of the bridge - for example:
			Serial Rx -> SerialReadUpStream -> _upStreamRead -> downstream_buffer -> _downstreamWrite -> SerialWriteDownStream -> Serial Tx
			Serial Tx <- SerialWriteUpStream <- _upStreamWrite <- upstream_buffer <- _downstreamRead <- SerialReadDownStream <- Serial Rx
*/

#include <Arduino.h>
#include "SerialBridge.h"

/* ----------------------------------------
   CONSTRUCTOR, DESTRUCTOR, STARTUP, UPDATE
   ----------------------------------------*/

SerialBridgeConfig SerialBridge::upstream;
SerialBridgeConfig SerialBridge::downstream;
SerialBuffer SerialBridge::downstreamBuffer;
SerialBuffer SerialBridge::upstreamBuffer;
bool SerialBridge::_interruptGuard = false;
bool SerialBridge::_streamIsDownStream = false;
uint8_t SerialBridge::_encodeGuard = 0;
int16_t SerialBridge::_report = SB_INVALID16;

SerialBridge::SerialBridge ( )
{
    // Set the default update handlers ('null' is a dummy function that does nothing)
    _updateUpstream = &SerialBridge::null;
    _updateDownstream = &SerialBridge::null;
}

SerialBridge::~SerialBridge()
{
    ;	// todo
}

void SerialBridge::startUpstream ( uint8_t mode, bool encode, HardwareSerial *hwserial, uint8_t address )
{
    // store config info.
    upstream.mode = mode;
    upstream.encode = encode;
    upstream.serial = hwserial;
    upstream.slaveAddress = address;
    upstream.rxbuffer = &downstreamBuffer;
    upstream.txbuffer = &upstreamBuffer;

    if ( encode )
        _encodeGuard = SB_WRAPPER_SIZE * ( ( SB_BUFFER_SIZE / SB_PACKET_SIZE ) + 1 );

    // Set the mode-specific config for upstream comms
    switch ( upstream.mode )
    {
    case SB_SERIAL:
        _updateUpstream = &SerialBridge::serialUpstream;
        break;
    case SB_COMMAND:
        _streamIsDownStream = false;
        _updateUpstream = &SerialBridge::commandUpstream;
        break;
    case SB_LOOPBACK:
        _updateUpstream = &SerialBridge::loopbackUpstream;
        break;
    case SB_WIRE:
        Wire.begin ( address );					// set up wire interface as slave
        Wire.onReceive ( wireReceiveCallback );	// callback function for wire ISR
        Wire.onRequest ( wireRequestCallback );	// callback function for wire ISR
        _interruptGuard = true;					// guard against interrupts when encoding/decoding buffers
    case SB_STREAM:								// handled by direct function calls
        _streamIsDownStream = false;
    default:
        break;
    }
}

void SerialBridge::startDownstream ( uint8_t mode, bool encode, HardwareSerial *hwserial, uint8_t address )
{
    // store config info
    downstream.mode = mode;
    downstream.encode = encode;
    downstream.serial = hwserial;
    downstream.slaveAddress = address;
    downstream.rxbuffer = &upstreamBuffer;
    downstream.txbuffer = &downstreamBuffer;

    if ( encode )
        _encodeGuard = SB_WRAPPER_SIZE * ( ( SB_BUFFER_SIZE / SB_PACKET_SIZE ) + 1 );

    // Set the mode-specific config for upstream comms
    switch ( downstream.mode )
    {
    case SB_SERIAL:
        _updateDownstream = &SerialBridge::serialDownstream;
        break;
    case SB_COMMAND:
        _streamIsDownStream = true;
        _updateDownstream = &SerialBridge::commandDownstream;
        break;
    case SB_LOOPBACK:
        _updateDownstream = &SerialBridge::loopbackDownstream;
        break;
    case SB_WIRE:
        Wire.begin();							// set up wire interface as a master (TODO in theory can also be a slave)
        TWBR = SB_16MHZ_TWBR_200KHZ;			// two wire bitrate (clocked by the master)
        _updateDownstream = &SerialBridge::wireDownstream;
        break;
    case SB_STREAM:								// handled by direct function calls
        _streamIsDownStream = true;
    default:
        break;
    }
}

void SerialBridge::update()
{
    // Update upstream and downstream comms using pointer-to-member function
    ( this->*_updateUpstream ) ();
    ( this->*_updateDownstream ) ();
}

/* ----------------------------------------
   ENCODE, DECODE, CRC
   ----------------------------------------*/

uint16_t SerialBridge::encodeBuffer ( SerialBuffer *buffer, bool interruptGuard, uint16_t bytes )
{
    uint16_t encoded = 0, available;
    uint8_t payload, packet;

    UPRINT ( F ( "encode:" ) );

    // turn off interrupts if needed to avoid data being written into the buffer while encoding
    // TODO - change to either:
    // 1) a separate temporary buffer for encoding/decoding, or
    // 2) work out how to only disable I2C interrupts
    if ( interruptGuard )
    {
        noInterrupts();
        UPRINT ( F ( " interrupts=off" ) );
    }

    // set initial value
    available = buffer->available();

    // check if bytes to encode, otherwise move on
    if ( available > 0 )
    {
        // encode the required number of packets
        while ( available > 0 )
        {
            // encode the lesser of:
            // 1) bytes available
            // 2) bytes specified to encode, or
            // 3) max bytes allowed per packet
            payload = min ( available, bytes );
            payload = min ( payload, SB_PAYLOAD_SIZE );

            packet = encodePacket ( buffer, payload );
            encoded += packet;
            available -= payload;
            bytes -= payload;

            USTREAM ( F ( " payload=" ) << payload );
            USTREAM ( F ( " encoded=" ) << encoded );
            USTREAM ( F ( " available=" ) << available );

            // break if there is an error encoding - should
            // only be caused by a buffer overflow
            if ( packet == 0 )
                break;
        }

        // wrap around any leftover bytes not encoded (only relevant to single packet mode)
        wrapBuffer ( buffer, available );
    }

    // turn on interrupts
    if ( interruptGuard )
    {
        interrupts();
        UPRINT ( F ( " interrupts=on" ) );
    }
    USTREAM ( endl );

    return encoded;
}

uint8_t SerialBridge::encodePacket ( SerialBuffer *buffer, uint8_t payloadsize )
{
    uint8_t cs = 0;

    // check there is data in the buffer and there is enough space
    // there must be at least 3 bytes space per packet for SOH, length, CRC
    if ( payloadsize == 0 || buffer->space() < SB_WRAPPER_SIZE )
        return 0;

    // add a header and the number of encoded bytes (excluding header and checksum)
    buffer->put ( SB_SOH );
    buffer->put ( payloadsize );

    UPRINT ( F ( " data=" ) );
    UPRINTCHAR ( SB_SOH );
    UPRINTCHAR ( payloadsize );

    // add data until a packet is filled
    for ( uint8_t i = 0; i < payloadsize; i++ )
    {
        cs += buffer->peek();
        UPRINTCHAR ( buffer->peek() );
        buffer->put ( buffer->get() );
    }

    // add checksum
    buffer->put ( cs );
    USTREAM ( F ( " cs=" ) << cs );

    return payloadsize + SB_WRAPPER_SIZE;
}

uint16_t SerialBridge::decodeBuffer ( SerialBuffer *buffer, bool interruptGuard , bool wholePacket )
{
    uint16_t decoded = 0, discarded = 0, available;
    uint8_t cs, header, payload;

    UPRINT ( F ( "decode:" ) );

    // turn off interrupts
    if ( interruptGuard )
    {
        noInterrupts();
        UPRINT ( F ( " interrupts=off" ) );
    }

    available = buffer->available();

    // decode the next packet in the buffer
    // this code is stateless (i.e. does not assume there is valid data,
    // nor any knowledge of previous calls to this 'decode' function)
    while ( available > 0 )
    {
        // find the next header and enough data for a valid packet
        header = buffer->peek ( );
        payload = buffer->peekn ( 1 );

        // found a packet?
        if ( ( header == SB_SOH ) && ( available >= ( uint16_t ) ( payload + SB_WRAPPER_SIZE ) ) )
        {
            UPRINT ( F ( " data=" ) );
            UPRINTCHAR ( SB_SOH );
            UPRINTCHAR ( payload );

            // check the CS
            cs = 0;
            for ( uint16_t i = 0; i < payload; i++ )
            {
                UPRINTCHAR ( buffer->peekn ( i + 2 ) );
                cs += buffer->peekn ( i + 2 );
            }
            USTREAM ( F ( " cs=" ) << cs );

            if ( buffer->peekn ( payload + 2 ) == cs )
            {
                // success!
                buffer->get();								// drop the header byte
                buffer->get();								// drop the payload byte
                for ( uint16_t i = 0; i < payload; i++ )
                    buffer->put ( buffer->get() );			// wrap around the packet bytes
                buffer->get();								// drop the cs byte
                available -= ( payload + SB_WRAPPER_SIZE );	// advance the available position
                decoded += payload;							// add the number of bytes decoded

                USTREAM ( F ( " payload=" ) << payload );
                USTREAM ( F ( " decoded=" ) << decoded );
                USTREAM ( F ( " available=" ) << available );
            }
            else
                UPRINT ( F ( " (error)" ) );
        }
        else
        {
            // no packet found
            if ( wholePacket )
            {
                // are we expecting whole packets? (e.g. I2C/Wire)
                buffer->flush ( available );
                discarded = available;
                available = 0;
            }
            else
            {
                // otherwise discard the current byte and advance one
                buffer->get();
                available--;
                discarded++;
            }
        }
    }

    USTREAM ( F ( " discarded=" ) << discarded );

    // wrap any trailing bytes
    wrapBuffer ( buffer, available );

    // turn on interrupts
    if ( interruptGuard )
    {
        interrupts();
        UPRINT ( F ( " interrupts=on" ) );
    }
    USTREAM ( endl );

    return decoded;
}

void SerialBridge::wrapBuffer ( SerialBuffer *buffer, uint16_t bytes )
{
    USTREAM ( F ( " wrap=" ) << bytes );

    for ( uint16_t i = 0; i < bytes; i++ )
        buffer->put ( buffer->get() );
}

const unsigned char CRC7_POLYNOMIAL = 0x91;
uint8_t SerialBridge::CRC ( uint8_t crc, uint8_t value )
{
    crc ^= value;
    for ( uint8_t j = 0; j < 8; j++ )
    {
        if ( crc & 1 )
            crc ^= CRC7_POLYNOMIAL;
        crc >>= 1;
    }
    return crc;
}

/* ----------------------------------------
   STREAM
   ----------------------------------------*/

int SerialBridge::available()
{
    if ( _streamIsDownStream )
        return downstream.txbuffer->available();
    else
        return upstream.txbuffer->available();
}

int SerialBridge::read()
{
    if ( _streamIsDownStream )
        return downstream.txbuffer->get();
    else
        return upstream.txbuffer->get();
}

int SerialBridge::peek()
{
    if ( _streamIsDownStream )
        return downstream.txbuffer->peek();
    else
        return upstream.txbuffer->peek();
}

void SerialBridge::flush()
{
    upstreamBuffer.flush();
    downstreamBuffer.flush();
}

size_t SerialBridge::write ( uint8_t b )
{
    if ( _streamIsDownStream )
        return downstream.rxbuffer->put ( b );
    else
        return upstream.rxbuffer->put ( b );
}

/* ----------------------------------------
   SERIAL
   ----------------------------------------*/

void SerialBridge::serialUpstream()
{
    serialProcess ( upstream.rxbuffer, upstream.txbuffer, upstream.serial, upstream.encode );
}

void SerialBridge::serialDownstream()
{
    serialProcess ( downstream.rxbuffer, downstream.txbuffer, downstream.serial, downstream.encode );
}

void SerialBridge::serialProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer, HardwareSerial *serial, bool encode )
{
    uint16_t i;

    // tx data - transmit as many bytes are:
    // 1) available, and
    // 2) can be encoded in the current buffer (including headers)
    uint16_t bytes = txbuffer->available();
    if ( bytes )
    {
        if ( encode )
            bytes = encodeBuffer ( txbuffer, _interruptGuard );

        for ( i = 0; i < bytes; i++ )
            serial->write ( ( char ) txbuffer->get() );
    }

    // rx data - this gets data until the buffer is full, then
    // stops (the UART buffer may then fill up and possibly overflow)
    bytes = serial->available();
    if ( bytes )
    {
        bytes = min ( bytes, ( rxbuffer->space() - SB_WRAPPER_SIZE ) );

        for ( i = 0; i < bytes; i++ )
            rxbuffer->put ( ( uint8_t ) serial->read() );

        if ( encode )
            decodeBuffer ( rxbuffer, _interruptGuard );
    }
}

/* ----------------------------------------
   LOOPBACK
   ----------------------------------------*/

void SerialBridge::loopbackUpstream()
{
    loopbackProcess ( upstream.rxbuffer, upstream.txbuffer, upstream.encode );
}

void SerialBridge::loopbackDownstream()
{
    loopbackProcess ( downstream.rxbuffer, downstream.txbuffer, downstream.encode );
}

void SerialBridge::loopbackProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer, bool encode )
{
    // get the number of bytes to encode - note that if there is insufficient space
    // in the rxbuffer, the txbuffer and UART buffer (if applicable) will fill up
    uint16_t bytes = min ( txbuffer->available(), rxbuffer->space() );

    if ( bytes )
    {
        // encode tx data
        if ( encode )
            bytes = encodeBuffer ( txbuffer, _interruptGuard, bytes );

        // loopback tx to the rx buffer
        while ( bytes )
        {
            rxbuffer->put ( txbuffer->get() );
            bytes--;
        }

        // decode rx data
        if ( encode )
            bytes = decodeBuffer ( rxbuffer, _interruptGuard );
    }
}

/* ----------------------------------------
   COMMANDS
   ----------------------------------------*/

void SerialBridge::commandUpstream()
{
    commandProcess ( upstream.rxbuffer, upstream.txbuffer );
}

void SerialBridge::commandDownstream()
{
    commandProcess ( downstream.rxbuffer, downstream.txbuffer );
}

int16_t SerialBridge::commandProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer )
{
    int16_t result = SB_INVALID16;
    uint8_t command, data_msb, data_lsb;
    uint8_t address = _streamIsDownStream ? downstream.slaveAddress : upstream.slaveAddress;

    // decode the next command in the buffer - this code is
    // stateless (i.e. does not assume there is valid data,
    // nor any knowledge of previous calls to this function).
    // Any data not processed is left for the next iteration
    while ( txbuffer->available() >= SB_CMD_SIZE )
    {
        // find the next command - 3 checks are required to accept a valid command:
        if ( txbuffer->peek() == SB_SOC )								// first byte must be SB_SOC
        {

            if ( ( txbuffer->peekn ( 1 ) == address ) &&				// second byte must match the MCU address
                    ( txbuffer->peekn ( SB_CMD_SIZE - 1 ) == SB_EOC ) )	// last byte must be SB_EOC
            {
                txbuffer->get();							// SOH (discarded)
                txbuffer->get();							// address (discarded)
                command = txbuffer->get();					// command byte
                data_msb = txbuffer->get();					// data_msb byte
                data_lsb = txbuffer->get();					// data_lsb byte
                txbuffer->get();							// EOC (discarded)

                //huzzah!
                switch ( command )							// process the command
                {
                case SB_CMD_REPORT:							// receive a report of the pin state
                    _report =  ( int16_t ) ( ( ( uint16_t ) data_msb << 8 ) + ( uint16_t ) data_lsb );
                    USTREAM ( F ( "process: cmd=report msb=" ) << data_msb << F ( " lsb=" ) << data_lsb << F ( " result=" ) << _report << endl );
                    return _report;
                    break;

                case SB_CMD_PIN_MODE:						// set the mode
                    pinMode ( data_msb, data_lsb );			// data_msb = pin, data_lsb = INPUT, OUTPUT or INPUT_PULLUP
                    USTREAM ( F ( "process: cmd=mode pin=" ) << data_msb << F ( " mode=" ) << data_lsb << endl );
                    break;

                case SB_CMD_DIGITAL_READ:					// read the pin state - high or low
                    USTREAM ( F ( "process: cmd=digitalread pin=" ) << data_msb );
                    result = digitalRead ( data_msb );		// data_msb = pin
                    USTREAM ( F ( " result=" ) << result << endl );
                    data_msb = highByte ( result );
                    data_lsb = lowByte ( result );
                    commandSend ( address, SB_CMD_REPORT, data_msb, data_lsb );
                    break;

                case SB_CMD_DIGITAL_WRITE:					// set the pin state
                    digitalWrite ( data_msb, data_lsb );	// data_msb = pin, data_lsb = HIGH or LOW
                    USTREAM ( F ( "process: cmd=digitalwrite pin=" ) << data_msb << F ( " state=" ) << data_lsb << endl );
                    break;

                case SB_CMD_ANALOG_READ:					// read the pin state - 0-255 TODO CHECK RANGE
                    USTREAM ( F ( "process: cmd=analogread pin=" ) << data_msb );
                    result = analogRead ( data_msb );			// data_msb = pin
                    USTREAM ( F ( " result=" ) << result << endl );
                    data_msb = highByte ( result );
                    data_lsb = lowByte ( result );
                    commandSend ( address, SB_CMD_REPORT, data_msb, data_lsb );
                    break;

                case SB_CMD_ANALOG_WRITE:					// set the pin state
                    analogWrite ( data_msb, data_lsb );		// data_msb = pin, data_lsb = 0-255 TODO CHECK RANGE
                    USTREAM ( F ( "process: cmd=analogwrite pin=" ) << data_msb << F ( " state=" ) << data_lsb << endl );
                    break;

                case SB_CMD_ANALOG_REF:						// set the analog reference
                    analogReference ( data_msb );				// data_msb = mode
                    USTREAM ( F ( "process: cmd=analogreference mode=" ) << data_msb << endl );
                    break;

                case SB_CMD_PING:
                    USTREAM ( F ( "process: cmd=ping" ) << endl );
                    commandSend ( address, SB_CMD_REPORT, data_msb, data_lsb );
                    break;

                default:
                    USTREAM ( F ( "process cmd=unknown(" ) << command << F ( ") msb:" ) << data_msb << F ( " lsb=" ) << data_lsb << endl );
                    break;
                }
            }
        }
        else
            // discard the current byte and advance one
            txbuffer->get();
    }
    return SB_INVALID16;
}

int16_t SerialBridge::commandReport ( bool peek )
{
    int16_t temp = _report;
    if ( !peek )
        _report = (int16_t) SB_INVALID16;
    return temp;
}

int16_t SerialBridge::commandSend ( uint8_t address, uint8_t command, uint8_t data_msb, uint8_t data_lsb, uint32_t timeout )
{
    SerialBuffer *rxbuffer = ( _streamIsDownStream ? downstream.rxbuffer : upstream.rxbuffer );

    // put a 6-byte command on to the rx buffer
    rxbuffer->put ( SB_SOC );		// start of command
    rxbuffer->put ( address );		// MCU to be commanded
    rxbuffer->put ( command );		// command type
    rxbuffer->put ( data_msb );		// first value (optional / default is 0 if no value specified)
    rxbuffer->put ( data_lsb );		// second value (optional / default is 0 if no value specified)
    rxbuffer->put ( SB_EOC );		// end of command

    // debugging info
#ifdef DEBUG
    USTREAM ( F ( "command: cmd=" ) << command << F ( " msb=" ) << data_msb << F ( " lsb=" ) << data_lsb );
    uint16_t available = rxbuffer->available();
    USTREAM ( F ( " available=" ) << available << F ( " data=" ) );
    for ( uint16_t c = 0; c < available; c++ )
        UPRINTCHAR ( rxbuffer->peekn ( c ) );
    USTREAM ( F ( " timeout=" ) << timeout << endl );
#endif

    // wait for the returned data until timeout
    // keep updating upstream and downstream comms
    timeout += millis();
    while ( millis() < timeout && this->commandReport ( true ) == (int16_t) SB_INVALID16 )
        this->update();

    return this->commandReport();
}

void SerialBridge::PinMode ( uint8_t pin, uint8_t mode, uint8_t address )
{
    commandSend ( address, SB_CMD_PIN_MODE, pin, mode );
}

int16_t SerialBridge::DigitalRead ( uint8_t pin, uint8_t address )
{
    return commandSend ( address, SB_CMD_DIGITAL_READ, pin, 0, SB_TIMEOUT );
}

void SerialBridge::DigitalWrite ( uint8_t pin, uint8_t state, uint8_t address )
{
    commandSend ( address, SB_CMD_DIGITAL_WRITE, pin, state );
}

int16_t SerialBridge::AnalogRead ( uint8_t pin, uint8_t address )
{
    return commandSend ( address, SB_CMD_ANALOG_READ, pin, 0, SB_TIMEOUT );
}

void SerialBridge::AnalogWrite ( uint8_t pin, int value, uint8_t address )
{
    commandSend ( address, SB_CMD_ANALOG_WRITE, pin, ( uint8_t ) value );
}

void SerialBridge::AnalogReference ( uint8_t mode, uint8_t address )
{
    commandSend ( address, SB_CMD_ANALOG_REF, mode );
}

uint32_t SerialBridge::Ping ( uint8_t address )
{
    uint32_t timer = millis();
    int16_t result = commandSend ( address, SB_CMD_PING, 0, SB_CMD_PING, SB_TIMEOUT );
	if (result == SB_CMD_PING)
		return (millis() - timer);
	else
		return (uint32_t) 0xFFFFFFFF;
}

bool SerialBridge::Reset ( bool hardReset, uint8_t hardResetPin, uint8_t address )
{
	return false;
}

/* ----------------------------------------
   WIRE
   ----------------------------------------*/

void SerialBridge::wireReceiveCallback ( int numBytes )
{
    // receive the specified number of bytes
    for ( int i = 0; i < numBytes; i++ )
        upstream.rxbuffer->put ( Wire.read() );

    // remove the wrapper
    if ( upstream.encode )
        decodeBuffer ( upstream.rxbuffer );	// n.b. no need for interruptguard as already in an ISR
}

void SerialBridge::wireRequestCallback()
{
    /*
    if ( upstream.txbuffer->available() == 0 )
    {
        upstream.txbuffer->put ( 'h' );
        upstream.txbuffer->put ( 'e' );
        upstream.txbuffer->put ( 'l' );
        upstream.txbuffer->put ( 'l' );
        upstream.txbuffer->put ( 'o' );
    }
    */
    // tx data - transmit as many bytes are:
    // 1) available,
    // 2) fit into a packet, and
    // 3) are encoded into a packet
    // TODO what if there is insufficient room to encode a header?
    uint16_t bytes = upstream.txbuffer->available();
    uint8_t packet[SB_PACKET_SIZE];
    uint8_t i;

    if ( bytes )
    {
        if ( upstream.encode )
            bytes = encodeBuffer ( upstream.txbuffer ); // n.b. no need for interruptguard as already in an ISR
        else
            bytes = min ( bytes, SB_PACKET_SIZE );

        for ( i = 0; i < bytes; i++ )
            packet[i] = upstream.txbuffer->get();

        // send the encoded packet upstream
        Wire.write ( packet, bytes );
    }
}

void SerialBridge::wireDownstream()
{
    /*
    downstream.txbuffer->flush();
    downstream.txbuffer->put ( 'h' );
    downstream.txbuffer->put ( 'e' );
    downstream.txbuffer->put ( 'l' );
    downstream.txbuffer->put ( 'l' );
    downstream.txbuffer->put ( 'o' );
    */
    uint8_t i;

    // send any bufferred data downstream
    uint16_t bytes = downstream.txbuffer->available();
    if ( bytes )
    {
        // start the transmission
        Wire.beginTransmission ( downstream.slaveAddress );

        // encode the buffer with a wrapper
        if ( downstream.encode )
            bytes = encodeBuffer ( downstream.txbuffer ); // n.b. no need for interruptguard as already in an ISR
        else
            bytes = min ( bytes, SB_PACKET_SIZE );

        // send the packet
        for ( i = 0; i < bytes; i++ )
            Wire.write ( ( char ) downstream.txbuffer->get() );

        // end transmission
        Wire.endTransmission();
    }

    // ---------------------------------------------------
    // check if data has been received (NOTE requestFrom is blocking)
    if ( Wire.requestFrom ( ( uint8_t ) downstream.slaveAddress, ( uint8_t ) SB_PACKET_SIZE ) == 0 )
        return;	// no data

    // read data into the buffer
    while ( Wire.available() && downstream.rxbuffer->space() > SB_WRAPPER_SIZE )
        downstream.rxbuffer->put ( Wire.read() );

    // remove the wrapper and process the checksum
    decodeBuffer ( downstream.rxbuffer, _interruptGuard, true );
}
