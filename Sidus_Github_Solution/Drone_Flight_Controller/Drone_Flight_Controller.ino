/*
 Name:		Drone_Flight_Controller.ino
 Created:	11/10/2016 8:56:34 PM
 Author:	SIDUS
 Description: This is the main code for Drone_Flight_Controller Project
*/

#include <WiFi.h>

#include "Local_Agenda.h"
#include "Config.h"
#include "cMsgT01.h"

//Global Class Definitions
Agenda scheduler;
cMsgT01 MsgT01;


// the setup function runs once when you press reset or power the board
void setup() {

	Serial.begin(921600);
	initWifi();

	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);

	//Insert all tasks into scheduler
	scheduler.insert(test_task, 500000);

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

void initWifi()
{
/*
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	delay(200);
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(400);
	}
	Serial.println("");
	Serial.println("WiFi connected");
	Serial.println("IP address: ");
	Serial.println(WiFi.localIP());
	*/
	WiFi.disconnect(true);
}
