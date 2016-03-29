/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This class implements a simple FIFO (circular) buffer
*/

#include "SerialBuffer.h"

/// <summary>
/// Constructor
/// </summary>
SerialBuffer::SerialBuffer( )
{
    _buffer = ( byte* ) malloc ( sizeof ( byte ) * SB_BUFFER_SIZE );
    flush();
}

/// <summary>
/// Destructor
/// </summary>
SerialBuffer::~SerialBuffer()
{
    free ( _buffer );
}

/* TODO
int SerialBuffer::operator[] ( uint16_t n )
{
    if ( this->available() == 0 )
        return -1;
    else
        return _buffer[ ( _buffer_head + n ) % _size ];
}
*/

/// <summary>
/// Read the an 8 bit byte from the buffer, n bytes ahead, without de-queueing
/// NOTE this will still return a value even if the buffer is empty
/// </summary>
/// <returns></returns>
uint8_t SerialBuffer::peek ( )
{
    // Read from "head"
    if ( _buffer_head == _buffer_tail )
        return 0;
    else
        return _buffer[ _buffer_head & SB_BUFFER_SIZE_MASK ];
}

/// <summary>
/// Read the an 8 bit byte from the buffer, n bytes ahead, without de-queueing
/// NOTE this will still return a value even if the buffer is empty
/// </summary>
/// <returns></returns>
uint8_t SerialBuffer::peekn ( uint16_t n )
{
    if ( _buffer_head == _buffer_tail )
        return 0;
    else
        return _buffer[ ( _buffer_head + n ) & SB_BUFFER_SIZE_MASK];
}

/// <summary>
/// Get the next byte from the queue
/// </summary>
/// <returns>value of next byte, or 0 if the buffer is empty</returns>
uint8_t SerialBuffer::get()
{
    // Empty buffer?
    if ( _buffer_head == _buffer_tail )
        return 0;
    else
    {
        _buffer_overflow = false;		// reset the overflow flag
        return _buffer[ ( _buffer_head++ ) & SB_BUFFER_SIZE_MASK];
    }
}

/// <summary>
/// Reset the buffer to zero
/// </summary>
void SerialBuffer::flush()
{
    _buffer_overflow = false;
    _buffer_head = _buffer_tail = 0;
}

/// <summary>
/// Remove the specified number of bytes from the buffer
/// </summary>
/// <param name="bytes">number of bytes to remove</param>
void SerialBuffer::flush ( uint16_t n )
{
    _buffer_overflow = false;
    _buffer_head += min ( this->available() , n );
}

/// <summary>
/// Put a byte in the queue
/// </summary>
/// <param name="b">byte value</param>
/// <returns>true of OK / false if buffer overflow</returns>
boolean SerialBuffer::put ( uint8_t b )
{
    uint8_t _space = SB_BUFFER_SIZE - ( ( _buffer_tail - _buffer_head ) & SB_BUFFER_SIZE_MASK );

    switch ( _space )
    {
    case 0:
        return !_buffer_overflow;
        break;

    case 1:
        _buffer_overflow = true;
    default:
        _buffer[ ( _buffer_tail++ ) & SB_BUFFER_SIZE_MASK] = b;
        return !_buffer_overflow;
        break;
    }
}

/// <summary>
/// Get the current number of bytes in the buffer
/// </summary>
/// <returns>number of bytes available</returns>
uint16_t SerialBuffer::available()
{
    return ( _buffer_tail - _buffer_head ) & SB_BUFFER_SIZE_MASK;
}

uint16_t SerialBuffer::size()
{
    return SB_BUFFER_SIZE;
}

uint16_t SerialBuffer::space()
{
    return SB_BUFFER_SIZE - ( ( _buffer_tail - _buffer_head ) & SB_BUFFER_SIZE_MASK );
}

/// <summary>
/// Has the buffer overflowed?
/// </summary>
/// <returns>true if buffer has overflowed / false if OK</returns>
boolean SerialBuffer::overflow()
{
    return _buffer_overflow;
}
