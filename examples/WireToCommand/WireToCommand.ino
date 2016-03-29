/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This is a demonstration of commanding a slave arduino over I2C

Usage:		This is the 'downstream' (slave) side of an arduino pair connected over I2C.
			Connect an Arduino pair as per the Configuration shown below. Upload the
			'CommandToWire example to the master Arduino, and 'WireToCommand' example to the slave
			Arduino. After it is uploaded start the serial terminal on the master Arduino to see
			the output.

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
    // initialise each side of the bridge. The default
    // slave address for the wire interface is 0x08
    myBridge.startUpstream ( SB_WIRE, true );
    myBridge.startDownstream ( SB_COMMAND );
}

void loop()
{
    // call the update function regularly to avoid buffer overrun
    myBridge.update();

    // add your program code here
}

