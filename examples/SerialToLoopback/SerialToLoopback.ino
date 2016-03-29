/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This is a demonstration of looping back the serial port on itself (like an old-
			fashioned RS-232 loopback plug).

Usage:		Just plug in a USB cable to an arduino, build and upload the sketch.
			After it is uploaded start the serial terminal - anything entered will be looped back

Configuration:
                ARDUINO
				Serial   ----------------------
					Rx ->| Pin 0 (loop to Tx) |
					Tx <-| Pin 1 (loop to Rx) |
						 ----------------------
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
    Serial.print ( F ( "Start..." ) );

    // Uncomment one option to get started:

    // option 1 - loopback without encoding
    //myBridge.startUpstream ( SB_SERIAL, false, &Serial );
    //myBridge.startDownstream ( SB_LOOPBACK );

    // option 2 - loopback with encoding
    myBridge.startUpstream(SB_SERIAL, false, &Serial);
    myBridge.startDownstream ( SB_LOOPBACK, true );
}

void loop()
{
    // call the update function regularly to avoid buffer overrun
    myBridge.update();

    // add some more program code here
	delay(1);
}
