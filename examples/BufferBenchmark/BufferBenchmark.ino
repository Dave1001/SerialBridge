/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This is used for development to test the efficiency of buffer read/write/encoding/decoding. 
			It benchmarks the time to run 1000 iterations of a data string through a loopback.

Usage:		Just plug in a USB cable to an arduino, build and upload the sketch.
			After it is uploaded, open a serial monitor at 115200baud to see the output

Configuration:

			ARDUINO    ----------------------
			USB { Rx ->| Pin 0 (loop to Tx) |
			      Tx <-| Pin 1 (loop to Rx) |
                       ----------------------
Output:
			Benchmark figures for an Arduino ATMEGA2560 running 1000 iterations
			of 24 bytes including encoding are:

			Original code									= 5701 millis
			Rewrote SerialBuffer (2's complement)			= 1288 millis ( huge improvement )
			Remove check for empty buffer					= 1276 millis (-12 millis / reverted)
			Changed from CRC to additive checksum			= 1012 millis (-264 millis)
			Change to using stream functions (write, read)	= 1040 millis (+28 millis)
			Reworked logic for processing serial, loopback  =  999 millis (-41 millis)
			Improved overflow handling of loopback, serial  = 1002 millis (+3 millis )
			Added flag for SB_STREAM & SB_CMD downstream    = 1025 millis (+23 millis / current code)
			Turn off encoding								=  474 millis
*/
#define SERIAL_BAUD		115200

#include <Arduino.h>
#include <Utility.h>
#include <Streaming.h>		
#include <SerialBridge.h>

/*
Declare the bridge then define the upstream and downstream communications types - choices are:
- SB_STREAM     - normal stream class to interact with your main program code (only works for UPSTREAM side)
- SB_WIRE       - wire (I2C) interface mainly for sending data between two boards
- SB_SERIAL     - hardware uart interface
- SB_COMMAND	- remotely command another MCU (e.g. to set/read GPIO pins)
- SB_LOOPBACK   - loops back data from rx to tx (like an old-fashioned RS-232 loopback plug)

Most types can be mixed and matched, EXCEPT each type can only be upstream OR downstream (NOT both on the
same arduino) - for example:

This is OK (upstream and downstream are different types):
	myBridge.startUpstream ( SB_STREAM );
	myBridge.startDownstream ( SB_LOOPBACK );

This is NOT OK (upstream and downstream are same types):
	myBridge.startUpstream ( SB_STREAM );
	myBridge.startDownstream ( SB_STREAM );
*/
SerialBridge myBridge;

void setup()
{
    // start the hardware serial objects
    Serial.begin ( SERIAL_BAUD );

    // start the bridge
    myBridge.startUpstream ( SB_STREAM, false );
    myBridge.startDownstream ( SB_LOOPBACK, true );
}

void loop()
{
    Serial << F ( "<start>" ) << endl;

	// Test string
    const char testString[] = "decode the next packet in the buffer this code is stateless (i.e. does not assume there is valid data, nor any knowledge of previous calls to this 'decode' function)";
    const uint16_t testStringSize = 24; // actual strlen is about 165 bytes
    const uint16_t iterations = 1000;
    uint32_t sentBytes = 0;
    uint32_t receivedBytes = 0;
    uint32_t endMillis = 0;

    // iterate
    uint32_t startMillis = millis();
    for ( uint16_t i = 0; i < iterations; i++ )
    {
        // send data
        myBridge.write ( testString, testStringSize );
        sentBytes+= testStringSize;

        // call the update function regularly to avoid buffer overrun
        myBridge.update();

        // read data
        while ( myBridge.available() )
        {
            myBridge.read();
			receivedBytes++;
        }
    }
    endMillis = millis();

    // print result
    Serial << F ( " start=" ) << startMillis << F ( " end=" ) << endMillis << F ( " duration=" ) << endMillis - startMillis << endl;
    Serial << F ( " testSize=" ) << testStringSize << F ( " sent=" ) << sentBytes << F ( " read=" ) << receivedBytes << F ( " lost=" ) << sentBytes - receivedBytes << endl;
    Serial << F ( " iterations=" ) << iterations << F ( " millis per iteration=" ) << ( endMillis - startMillis ) / iterations << endl << F ( "<end>" ) << endl;
}

