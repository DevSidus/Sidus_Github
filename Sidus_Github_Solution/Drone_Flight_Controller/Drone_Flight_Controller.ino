/*
 Name:		Drone_Flight_Controller.ino
 Created:	11/10/2016 8:56:34 PM
 Author:	SIDUS
*/

#include "Agenda2.h"
//Global Declarations
Agenda2 scheduler;
int counter = 0;
int ledPin = 5;


// the setup function runs once when you press reset or power the board
void setup() {

	pinMode(ledPin, OUTPUT);

	scheduler.insert(test_task, 1000000);

}

// the loop function runs over and over again until power down or reset
void loop() {
  scheduler.update();
}

void test_task()
{
	counter++;
	if (counter % 2 == 0)
		digitalWrite(ledPin, HIGH);
	else
		digitalWrite(ledPin, LOW);
}
