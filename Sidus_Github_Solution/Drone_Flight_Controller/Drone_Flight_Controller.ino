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
cMsgR01 MsgR01;
cSerialParse serialParse(sizeof(MsgT01.message), MsgT01.message.startChar1, MsgT01.message.startChar2, MsgT01.message.endChar);
HardwareSerial Serial2(2);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(921600);
	
	pinMode(16, INPUT);
	pinMode(17, OUTPUT);


	Serial2.begin(921600);
	
	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	

	//Insert all tasks into scheduler
	scheduler.insert(test_task, 500000);
	scheduler.insert(serialCheck, 15000);

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
	int numberofbytes = Serial2.available();
	if (numberofbytes > 0)
	{
		unsigned char buffer[sizeof(MsgT01.message)];
		Serial2.readBytes(buffer, numberofbytes);
		serialParse.Push(buffer, numberofbytes);
		//Serial.write(buffer, numberofbytes);
		if (serialParse.getParsedData(MsgT01.dataBytes, sizeof(MsgT01.message)))
		{
			MsgT01.setPacket();
			Serial.println(MsgT01.message.mpuPitch * 180 / M_PI);
		}
	}
}
