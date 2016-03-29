/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This class implements a simple FIFO (circular) buffer
*/

#ifndef _SERIALBUFFER_h
#define _SERIALBUFFER_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "arduino.h"
#else
#include "WProgram.h"
#endif

#define SB_BUFFER_SIZE		 (uint16_t) 64						// buffer size in bytes - MUST be a power of 2
#define SB_BUFFER_SIZE_MASK  (uint16_t) (SB_BUFFER_SIZE - 1)	// don't edit this or the buffer will break

class SerialBuffer
{
private:
    boolean _buffer_overflow;
    uint8_t *_buffer;
    volatile uint16_t _buffer_tail;
    volatile uint16_t _buffer_head;

public:
	SerialBuffer();
    ~SerialBuffer();
    // int operator[] ( uint16_t n ); // TODO
    uint8_t peek ();
	uint8_t peekn(uint16_t pos = 0);
	uint8_t get();
    boolean put ( uint8_t byte );
    boolean overflow();
    uint16_t available();
    uint16_t size();
    uint16_t space();
    void flush();
    void flush ( uint16_t bytes );

	/* TODO
    operator bool()
    {
        return !_buffer_overflow;
    } */
};

#endif

