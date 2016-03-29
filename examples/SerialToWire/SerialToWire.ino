/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This is a demonstration of bridging USB to the two-wire interface

Usage:		This is the 'upstream' (master) side of an arduino pair connected over I2C.
			Connect an Arduino pair as per the Configuration shown below. Upload the
			'SerialToWire example to the master Arduino, and 'WireToLoopback' example to the slave
			Arduino. After it is uploaded start the serial terminal on the master Arduino -
			anything entered will be immediately looped back through through the slave Arduino
			and should appear back in the in the terminal window of the master.

Configuration:

           ARDUINO 1 (Master)                          ARDUINO 2 (Slave)
         ----------------------                      ----------------------
         |             Pin A5 | <-----  SCL   -----> | Pin A5              |
 USB <-> | (Stream)    Pin A4 | <-----  SDA   -----> | Pin A4   Loopback   |
         |             Gnd    | <----- Ground -----> | Gnd                 |
		 |             5V     | <-----   5V   -----> | 5V                  |
		 ----------------------                      -----------------------
*/

#include <Arduino.h>
#include <SerialBridge.h>

#define SERIAL_BAUD		115200

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
    Serial.println ( F ( "Start..." ) );

    // Set up the serial and wire interface (with encoding)
    myBridge.startUpstream (SB_SERIAL, false, &Serial );
    myBridge.startDownstream ( SB_WIRE, true );
}

void loop()
{
    // call the update function regularly to avoid buffer overrun
    myBridge.update();

    // add your program code here
}
