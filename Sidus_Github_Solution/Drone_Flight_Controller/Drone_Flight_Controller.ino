/*
 Name:		Drone_Flight_Controller.ino
 Created:	11/10/2016 8:56:34 PM
 Author:	SIDUS
 Description: This is the main code for Drone_Flight_Controller Project
*/

//Local include class and files
#include "Local_Agenda.h"
#include "Config.h"
#include "cMsgT01.h"
#include "cMsgR01.h"
#include "cSerialParse.h"

//Global Class Definitions
Agenda scheduler;
cMsgT01 MsgT01;
cSerialParse serialParse(SERIAL_PACKET_SIZE, SERIAL_START_CHAR_1, SERIAL_START_CHAR_2, SERIAL_END_CHAR);


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(921600);
	
	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	

	//Insert all tasks into scheduler
	scheduler.insert(test_task, 500000);
	scheduler.insert(serialCheck, 50000);

}

// the loop function runs over and over again until power down or reset
void loop() {
	//Just call scheduler update and let it do all the process
	scheduler.update();	
}

void test_task()
{
	test_task_counter++;
	if (test_task_counter % 2 == 0)
	{
		digitalWrite(PIN_LED, HIGH);
	}
	else
	{
		digitalWrite(PIN_LED, LOW);
	}
}

void serialCheck()
{
	int numberofbytes = Serial.available();
	if (numberofbytes > 0)
	{
		unsigned char buffer[SERIAL_PACKET_SIZE];
		Serial.readBytes(buffer, numberofbytes);
		serialParse.Push(buffer, numberofbytes);
		Serial.write(buffer, numberofbytes);
		if (serialParse.getParsedData(buffer, SERIAL_PACKET_SIZE))
		{
			Serial.println("tamam");
		}
	}
}
