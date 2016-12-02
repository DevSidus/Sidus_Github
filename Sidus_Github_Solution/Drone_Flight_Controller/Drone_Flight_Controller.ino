/*
 Name:		Drone_Flight_Controller.ino
 Created:	11/10/2016 8:56:34 PM
 Author:	SIDUS
 Description: This is the main code for Drone_Flight_Controller Project
*/

#include <WiFi.h>
#include <Wire.h>
#include <MPU6050_9Axis_MotionApps41.h>

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

MPU6050 mpu;
bool mpuStatus;


// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(921600);



	initWifi();

	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	//pinMode(PIN_MPU_POWER_ON, OUTPUT);

	//digitalWrite(PIN_MPU_POWER_ON, LOW);

	//delay(200);
	Wire.begin();
	//Wire.setClock(800000L);


	//Insert all tasks into scheduler
	scheduler.insert(test_task, 500000);
	scheduler.insert(serialCheck, 50000);
	scheduler.insert(initMPU, 2000000);

	//delay(200);
	//initMPU();
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

void initMPU()
{
	//digitalWrite(PIN_MPU_POWER_ON, LOW);
	//delay(200);
	//digitalWrite(PIN_MPU_POWER_ON, HIGH);
	//delay(200);

	// initialize device
	//Serial.println(F("Initializing I2C devices..."));

	Wire.beginTransmission(104);

	int error = Wire.endTransmission();
	if (error == 0)
	{
		Serial.println("MPU6050 found at 0x68");
	}
	else
	{
		Serial.println("MPU6050 could not be found at 0x68");
	}

	mpu.initialize();
	delay(200);
	if (mpu.testConnection())
	{
		Serial.println("MPU6050 connection successful");
	}
	else
	{
		mpuStatus = false;
		Serial.println("MPU6050 connection failed");
		//return false;
	}
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	// supply your own gyro offsets here, scaled for min sensitivity
	/*
	mpu.setXGyroOffset(153);
	mpu.setYGyroOffset(-60);
	mpu.setZGyroOffset(18);
	mpu.setXAccelOffset(-2714);
	mpu.setYAccelOffset(-714);
	mpu.setZAccelOffset(1551);
	*/

	// make sure it worked (returns 0 if so)
	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();

		mpuStatus = true;
	}
	else
	{

		mpuStatus = false;
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	
}