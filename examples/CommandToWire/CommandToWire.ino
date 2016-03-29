/*
Date:		February 2016

Repository:	https://github.com/Dave1001/SerialBridge

License:	GNU Lesser General Public License as published by the Free Software Foundation version 2.1

About:		This is a demonstration of commanding a slave arduino over I2C

Usage:		This is the 'upstream' (master) side of an arduino pair connected over I2C.
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
unsigned long updatetime = 0;
uint8_t pinState = LOW;

void setup()
{
    // start the hardware serial object
    Serial.begin ( SERIAL_BAUD );
    Serial.println ( F ( "<setup>" ) );

    // Set up the bridge for commands and I2C(wire)
    myBridge.startUpstream ( SB_COMMAND );
    myBridge.startDownstream ( SB_WIRE, true );

	// check the remote node is responding
	uint32_t timer = myBridge.Ping();
	Serial.print(F("Ping millis= "));
	Serial.println(timer);

    // set up a GPIO pin - for compatibility, the syntax is as close
    // as possible to normal arduino digitalPin read/write commands.
    myBridge.PinMode ( 13, OUTPUT );
}

void loop()
{
    int result;

    // call the update function regularly to avoid buffer overrun
    myBridge.update();

    // toggle the pin state every 1 second
    if ( millis() > updatetime )
    {
        // toggle the pin state
        Serial.println ( F ( "<start>" ) );
        pinState = pinState == LOW ? HIGH : LOW;

        /*
        // --------------------------------------
        // example of how to set an analog pin state
        myBridge.AnalogWrite ( A0, pinState == LOW ? 0 : 255 );
        Serial.print(F("AnalogWrite pin A0 = "));
        Serial.println(pinState == LOW ? 0 : 255);

        // --------------------------------------
        // example of how to read an analog pin state
        result = myBridge.AnalogRead ( A0 );
        Serial.print(F("AnalogRead pin A0 = "));
        Serial.println(result);
        */
        // --------------------------------------
        // example of how to set a digital pin state
        myBridge.DigitalWrite ( 13, pinState );
        Serial.print ( F ( "DigitalWrite pin 13 = " ) );
        Serial.println ( pinState == LOW ? F ( "LOW" ) : F ( "HIGH" ) );

        // --------------------------------------
        // example of how to read a digital pin state
        result = myBridge.DigitalRead ( 13 );
        Serial.print ( F ( "DigitalRead pin 13 = " ) );
        if ( result == SB_INVALID8 )
            Serial.println ( F ( "invalid" ) );
        else
            Serial.println ( result );

        // --------------------------------------
        // set the next update time
        updatetime = millis() + 1000;
        Serial.println ( F ( "<end>" ) );
    }

    // add your program code here
    delay ( 1 );
}
