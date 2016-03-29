// TODO declare this as a singleton (currently you can't have more than one / static buffers make this impossible)
// TODO work out how to have multiple slaves

/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This class bridges serial IO between a master (upstream) and slave (downstream)
*/

#ifndef _SERIALBRIDGE_h
#define _SERIALBRIDGE_h

#ifndef DEBUG
//#define DEBUG
#endif

#include <Arduino.h>
#include <inttypes.h>
#include <Wire.h>
#include <Utility.h>

#ifdef DEBUG
 #include <Stream.h>
#endif


#include "SerialBuffer.h"

// ----------------------------------------
// DEFINITIONS
// ----------------------------------------

// Mode flags used to define input and outputs
#define SB_SERIAL			(uint8_t) 0			// hardware UART 
#define SB_WIRE				(uint8_t) 1			// two wire I2C (pins A4 and A5 on arduino uno)
#define SB_COMMAND			(uint8_t) 2			// send commands to another MCU (e.g. turn on/off gpio pins)
#define SB_STREAM			(uint8_t) 3			// stream interface (e.g. makes write, flush etc available to your application code)
#define SB_LOOPBACK			(uint8_t) 4			// loopback comms on itself (used for testing)

// Commands that can be sent from a master to an addressable slave
// These commands are all designed to be initialised and commanded 
// remotely - simply set up the slave using the WireToCommand or
// SerialToCommand sketches (the slave is stateless and has no need
// to have any prior configuration of states, pins etc) - the only 
// parameter it needs is an address.
#define SB_CMD_REPORT			(uint8_t) 0x00		// report from the remote MCU
#define SB_CMD_DATA				(uint8_t) 0x01		// command to send a data packet ( currently not used )
#define SB_CMD_PIN_MODE			(uint8_t) 0x02		// command to set the remote pin mode 
#define SB_CMD_DIGITAL_READ	    (uint8_t) 0x03		// command to read the remote pin digital state 
#define SB_CMD_DIGITAL_WRITE	(uint8_t) 0x04		// command to set the remote pin digital state 
#define SB_CMD_ANALOG_READ		(uint8_t) 0x05		// command to read the remote pin analog state 
#define SB_CMD_ANALOG_WRITE		(uint8_t) 0x06		// command to set the remote pin analog state 
#define SB_CMD_ANALOG_REF		(uint8_t) 0x07		// command to set the remote pin analog reference 
#define SB_CMD_SHIFT_MODE		(uint8_t) 0x08		// command to set the slave pins as a shift register
#define SB_CMD_SHIFT_WRITE		(uint8_t) 0x09		// command to write a byte to the shift register
#define SB_CMD_PING				(uint8_t) 0x0A		// command to check if the remote Arduino is alive
#define SB_CMD_RESET			(uint8_t) 0x0B		// command to perform a remote reset 

#define SB_SLAVE_ADDRESS	(uint8_t) 0x08 	    // default slave address, any number from 0x01 to 0x7F

#define SB_INVALID8			(uint8_t) 0xFF			// flag for any 8 bit value that hasn't been set
#define SB_INVALID16		(uint16_t) 0xFFFF		// flag for any 16 bit value that hasn't been set
#define SB_INVALID32        (uint32_t) 0xFFFFFFFF	// flag for any 32 bit value that hasn't been set
#define SB_ALL				(uint16_t) 0xFFFF	// flag to encode all bytes in the buffer
#define SB_SOH				(uint8_t) 0x24		// byte used to indicate start of header (ascii '$')
#define SB_SOC				(uint8_t) 0x23		// byte used to indicate start of command (ascii '#')
#define SB_EOC				(uint8_t) 0x20		// byte used to indicate end of command (ascii <space>)
#define SB_TIMEOUT			(uint16_t) 200		// timeout in milliseconds for commands where a reply is expected
#define SB_CMD_SIZE			(uint8_t) 6         // size of each command in bytes (always fixed length)
#define SB_PACKET_SIZE		(uint8_t) 32		// maximum number of bytes to transfer at a time - note max for I2C is 32 bytes
#define SB_WRAPPER_SIZE		(uint8_t) 3			// encoding adds a 3 byte wrapper for header and checksum
#define SB_PAYLOAD_SIZE		SB_PACKET_SIZE - SB_WRAPPER_SIZE	// size of the payload within a packet (do not edit)

// Two Wire Bit Rate Frequency - refer to here -> http://gammon.com.au/i2c
/*
	The formula for converting TWBR into frequency is :
	freq = clock / (16 + (2 * TWBR * prescaler))

	The default prescaler is 1, and the default value for TWBR(on the Uno etc.) is 72. Thus:
	freq = 16000000 / (16 + 144) = 100000

	TWBR   prescaler   Frequency
	12       1       400   kHz(the maximum supported frequency)
	32       1       200   kHz
	72       1       100   kHz(the default)
	152      1       50    kHz
	78       4       25    kHz
	158      4       12.5  kHz

	400 kHz is the maximum for Atmega328P running at 16 MHz (eg. Arduino Uno, Duemilanove, etc.)

	To set the prescaler to 4 you need to set the bit TWPS0 in TWSR, so for example to have a clock of 12.5 kHz:
	Wire.begin();
	TWBR = 158;
	TWSR |= bit(TWPS0);
*/
#define SB_16MHZ_TWBR_400KHZ	12
#define SB_16MHZ_TWBR_200KHZ	32
#define SB_16MHZ_TWBR_100KHZ	72
#define SB_16MHZ_TWBR_50KHZ		158

/// <summary>A serial bridge configuration struct.</summary>
struct SerialBridgeConfig
{
    /// <summary>serialbridge mode.</summary>
    uint8_t mode = SB_INVALID8;

    /// <summary>flag to encode the buffer before sending.</summary>
    bool encode = false;

    /// <summary>serial object (if applicable)</summary>
    HardwareSerial *serial = NULL;

    /// <summary>address for the wire slave (used by both upstream and downstream sides)</summary>
    uint8_t slaveAddress = SB_INVALID8;

    /// <summary>pointer to buffer for the stream to read into (incoming data)</summary>
    SerialBuffer *rxbuffer;

    /// <summary>pointer to the buffer to write from (outgoing data)</summary>
    SerialBuffer *txbuffer;
};

// --------------------------------------------------

/// <summary>Serial bridge class.</summary>
class SerialBridge : public Stream
{
public:
    /// <summary>Upstream configuration.</summary>
    static SerialBridgeConfig upstream;

    /// <summary>Downstream configuration.</summary>
    static SerialBridgeConfig downstream;

    /// <summary>Upstream buffer (Slave->Master)</summary>
    static SerialBuffer upstreamBuffer;

    /// <summary>Downstream buffer (Master->Slave)</summary>
    static SerialBuffer downstreamBuffer;

	/// <summary>Default constructor.</summary>
	SerialBridge();

	/// <summary>Destructor.</summary>
	~SerialBridge();

	/// <summary>Configure the upstream side of the bridge.</summary>
	/// <param name="mode">	   SB_SB_SERIAL, SB_WIRE, SB_COMMAND, SB_STREAM, or SB_LOOPBACK.</param>
	/// <param name="encode">  true to encode.</param>
	/// <param name="hwserial">[in,out] (Optional) pointer to hardwareserial.</param>
	/// <param name="address"> address of this node.</param>
	void startUpstream(uint8_t mode, bool encode = false, HardwareSerial *hwserial = NULL, uint8_t address = SB_SLAVE_ADDRESS);

	/// <summary>Configure the downstream side of the bridge.</summary>
	/// <param name="mode">	   SB_SB_SERIAL, SB_WIRE, SB_COMMAND, SB_STREAM, or SB_LOOPBACK.</param>
	/// <param name="encode">  true to encode.</param>
	/// <param name="hwserial">[in,out] (Optional) pointer to hardwareserial.</param>
	/// <param name="address"> address of the remote (downstream) node if applicable.</param>
	void startDownstream(uint8_t mode, bool encode = false, HardwareSerial *hwserial = NULL, uint8_t address = SB_SLAVE_ADDRESS);

	/// <summary>
	/// Update function. This must be called frequently to avoid buffer overrun. Some tuning of the
	/// buffersize, encoding, and speed this update function is called may be necessary depending on
	/// the speed and volume of data passing through the bridge.
	/// </summary>
	void update();

    /// <summary>Callback, called when the wire receives data.</summary>
    /// <param name="numBytes">Number of bytes received.</param>
    static void wireReceiveCallback ( int numBytes );	

    /// <summary>Callback, called when the wire requests data.</summary>
    static void wireRequestCallback();

    /// <summary>Gets the available bytes in the stream object.</summary>
    /// <returns>Number of bytes available.</returns>
    virtual int available();

    /// <summary>Gets the top-of-stack byte in the stream.</summary>
    /// <returns>The current top-of-stack byte.</returns>
    virtual int read();

    /// <summary>Returns the top-of-stack object without removing it.</summary>
    /// <returns>The current top-of-stack object.</returns>
    virtual int peek();

    /// <summary>Flushes this object.</summary>
    virtual void flush();

    /// <summary>Writes the given byte to the stream object.</summary>
    /// <param name="b">The byte to write.</param>
    /// <returns>TODO A size_t.</returns>
    virtual size_t write ( uint8_t b );

    /// <summary>Prints bytes to the stream object.</summary>
    using Print::write;
	
	/// <summary>
	/// Set the pin mode on a remote arduino. This works the same as the regular Arduino.h function-
	/// just a little slower due to the remote comms.
	/// </summary>
	/// <param name="pin">	  The pin.</param>
	/// <param name="mode">   The mode.</param>
	/// <param name="address">The address.</param>
	// TODO
	void PinMode(uint8_t pin, uint8_t mode, uint8_t address = SB_SLAVE_ADDRESS);
	void DigitalWrite(uint8_t pin, uint8_t state, uint8_t address = SB_SLAVE_ADDRESS);
	int16_t DigitalRead(uint8_t pin, uint8_t address = SB_SLAVE_ADDRESS);
	int16_t AnalogRead(uint8_t pin, uint8_t address = SB_SLAVE_ADDRESS);
	void AnalogWrite(uint8_t pin, int value, uint8_t address = SB_SLAVE_ADDRESS);
	void AnalogReference(uint8_t mode, uint8_t address = SB_SLAVE_ADDRESS);

	uint32_t Ping(uint8_t address = SB_SLAVE_ADDRESS);
	bool Reset(bool hardReset = false, uint8_t hardResetPin = SB_INVALID8, uint8_t address = SB_SLAVE_ADDRESS);

	// --------------------------------------------------
	
private:
    // private member data
    static bool _interruptGuard;
    static uint8_t 	_encodeGuard;
    static bool _streamIsDownStream;		// flag used by both stream and command functions to determine whether they are upstream or downstream
	static int16_t _report;

    // private pointer-to-member functions
    void null() // do nothing
    {
        return;
    };
    typedef void ( SerialBridge::*functionPointer ) ();
    functionPointer _updateUpstream;
    functionPointer _updateDownstream;

    /// <summary>Encode the buffer (put a wrapper around packets in the buffer)</summary>
    /// <param name="buffer">		 [in,out] The buffer to be encoded.</param>
    /// <param name="interruptGuard">Set to true to ensure encoding is not corrupted by interrupts.</param>
    /// <param name="bytes">		 Number of bytes to encode.</param>
    /// <returns>Number of bytes encoded (including wrapper) or 0 if failure.</returns>
    static uint16_t encodeBuffer ( SerialBuffer *buffer, bool interruptGuard = false, uint16_t bytes = SB_ALL );

	/// <summary>Encode a single packet (put a wrapper around a single data packet).</summary>
	/// <param name="buffer">	  [in,out] The buffer to be encoded.</param>
	/// <param name="payloadsize">Number of bytes to be encoded (payload size, excluding wrapper).</param>
	/// <returns>Number of bytes encoded (packet size, including wrapper)</returns>
	static uint8_t encodePacket(SerialBuffer *buffer, uint8_t payloadsize);

    /// <summary>Decode the buffer (remove the wrapper around each data packet).</summary>
    /// <param name="buffer">		 [in,out] The buffer to be decoded.</param>
    /// <param name="interruptGuard">Set to true to ensure encoding is not corrupted by interrupts.</param>
    /// <param name="wholePacket">Set to true if the buffer contains only whole encoded packets (e.g. for I2C/wire).</param>
    /// <returns>Number of bytes decoded (payload size, excluding wrapper) or 0 if failure.</returns>
    static uint16_t decodeBuffer ( SerialBuffer *buffer, bool interruptGuard = false, bool wholePacket = false);

    /// <summary>
    /// Wrap around any extra bytes in the buffer left over from encoding/decoding 
    /// Original buffer = 12435678____________ 
    /// Encoded buffer  = _____678SO12345C____ 
    /// Wrapped	buffer  = ________SO12345C678_.
    /// </summary>
    /// <param name="buffer">[in,out] The buffer to be wrapped.</param>
    /// <param name="bytes"> Number of bytes to be wrapped.</param>
    static void wrapBuffer ( SerialBuffer *buffer, uint16_t bytes );

    /// <summary>
    /// Add a byte to a running calculation of CRC (NOTE: CRC is no longer used for encoding as
    /// checksum is quicker.
    /// </summary>
    /// <param name="crc_value"> The existing CRC value.</param>
    /// <param name="byte_value">The byte value to be added.</param>
    /// <returns>The new CRC value.</returns>
    static uint8_t CRC ( uint8_t crc_value, uint8_t byte_value );

    /// <summary>
    /// Process serial comms upstream .
    /// </summary>
    void serialUpstream();

	/// <summary>
	/// Process serial comms downstream.
	/// </summary>
    void serialDownstream();

    /// <summary>
    /// Serial process that sends the bytes from the buffer to the serial output, and receives and
    /// buffers data from serial input.
    /// </summary>
    /// <param name="rxbuffer">[in,out] If non-null, the rxbuffer.</param>
    /// <param name="txbuffer">[in,out] If non-null, the txbuffer.</param>
    /// <param name="serial">  [in,out] If non-null, the serial UART.</param>
    /// <param name="encode">  True to encode/decode data.</param>
    void serialProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer, HardwareSerial *serial, bool encode = false );

    /// <summary>Loopback upstream.</summary>
    void loopbackUpstream();

    /// <summary>Loopback downstream.</summary>
    void loopbackDownstream();

    /// <summary>
    /// Loopback process that takes data from the buffer, encodes and decodes as applicable, and
    /// loops it back.
    /// </summary>
    /// <param name="rxbuffer">[in,out] If non-null, the rxbuffer.</param>
    /// <param name="txbuffer">[in,out] If non-null, the txbuffer.</param>
    /// <param name="encode">  True to encode/decode data.</param>
    void loopbackProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer, bool encode = false );

    /// <summary>Command upstream.</summary>
    void commandUpstream();

    /// <summary>Command downstream.</summary>
    void commandDownstream();

    /// <summary>
    /// Command process that takes data from the buffer, encodes and decodes as applicable, and
    /// parses received commands.
    /// </summary>
    /// <param name="rxbuffer">[in,out] If non-null, the rxbuffer.</param>
    /// <param name="txbuffer">[in,out] If non-null, the txbuffer.</param>
    /// <returns>An int16_t.</returns>
    int16_t commandProcess ( SerialBuffer *rxbuffer, SerialBuffer *txbuffer );

	/// <summary>
	/// Send a command, and (if applicable) wait for a response. Commands are always fixed length (6
	/// bytes) consisting of Start of Command (SOC) (one byte), Address (one byte),  
	/// ID (one byte), Data MSB (one byte), Data LSB (one byte), End of Command (EOC) (one byte).
	/// Checksum etc for commands is handled by encoding/decoding at the buffer level. If a response
	/// is requested, the command is blocking for SB_TIMEOUT.
	/// </summary>
	/// <param name="address"> The address to send the command to (default = SB_SLAVE_ADDRESS).</param>
	/// <param name="command"> The command (SB_CMD_XXX ).</param>
	/// <param name="data_msb">The data MSB.</param>
	/// <param name="data_lsb">The data LSB.</param>
	/// <param name="timeout"> Timeout to wait for a response in millis.</param>
	/// <returns>Returns either a response int16_t, or SB_INVALID if unable to read anything.</returns>
	int16_t commandSend ( uint8_t address, uint8_t command, uint8_t data_msb = 0, uint8_t data_lsb = 0, uint32_t timeout = 0);

	/// <summary>Special function used internally to poll for a Command response.</summary>
	/// <param name="peek">Set true to 'peek' without removing the byte</param>
	/// <returns>Returns either a response int16_t, or SB_INVALID if unable to read anything.</returns>
	int16_t commandReport( bool peek = false );

    /// <summary>Wire 'master' process that sends data and requests data from the downstream slave.</summary>
    void wireDownstream();
};

#endif