/*
 Name:		Drone_CoWorker.ino
 Created:	12/2/2016 6:37:46 PM
 Author:	SIDUS
*/

#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>


//Local include class and files
#include "Config_CoWorker.h"
#include "cMsgR01.h"
#include "cMsgT01.h"
#include "Local_Agenda.h"
#include "cSerialParse.h"
#include "Local_MS5611.h"
#include "Local_HMC5883L.h"

//Global Class Definitions
Agenda scheduler;
cMsgT01 MsgT01;
cMsgR01 MsgR01;
cSerialParse serialParse(sizeof(MsgR01.message), MsgR01.message.startChar1, MsgR01.message.startChar2, MsgR01.message.endChar);

MPU6050 mpu;
MS5611 barometer;
HMC5883L compass;

// the setup function runs once when you press reset or power the board
void setup() {

	//Start serial communication
	Serial.begin(921600);
	Wire.begin(PIN_MCU_SDA, PIN_MCU_SCL);
	Wire.setClock(800000L);


	Serial.println("Serial started");

	//Configure all PINs
	pinMode(PIN_MPU_POWER_ON, OUTPUT);
	digitalWrite(PIN_MPU_POWER_ON, LOW);


	//Insert all tasks into scheduler
	scheduler.insert(test_task, 500000);
	scheduler.insert(serialCheck, 50000);
	scheduler.insert(processMpuTask, 17000);
	scheduler.insert(serialTransmit, 20000);
	scheduler.insert(updateBarometerData, 19000);
	scheduler.insert(updateCompassData, 37000);

	initMPU();
	initBarometer();
	initCompass();
}

// the loop function runs over and over again until power down or reset
void loop() {
	//Just call scheduler update and let it do all the process
	scheduler.update();
}

void test_task()
{
	//Serial.println(compassHdg);
}

void serialCheck()
{
	int numberofbytes = Serial.available();
	if (numberofbytes > 0)
	{
		unsigned char buffer[sizeof(MsgR01.message)];
		Serial.readBytes(buffer, numberofbytes);
		serialParse.Push(buffer, numberofbytes);
		Serial.write(buffer, numberofbytes);
		if (serialParse.getParsedData(buffer, sizeof(MsgR01.message)))
		{
			//Serial.println("tamam");
		}
	}
}

bool initMPU()
{
	digitalWrite(PIN_MPU_POWER_ON, LOW);
	delay(100);
	digitalWrite(PIN_MPU_POWER_ON, HIGH);
	delay(100);

	// initialize device
	Serial.println(F("Initializing I2C devices..."));

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
		return false;
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

void processMpuTask()
{

	//get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();

	//If mpu connection is lost, re initialize mpu, check the code mpuIntStatus statement again!
	if (mpuIntStatus == 0 || !mpuStatus)
	{
		initMPU();
	}

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		//Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if ((mpuIntStatus & 0x02) && (fifoCount >= packetSize))
	{
		mpuProcessStartTime = micros();

		// wait for correct available data length, should be a VERY short wait
		//while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		//fifoCount -= packetSize;

		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetGravity(&gravity, &q);
		mpu.dmpGetAccel(&aa, fifoBuffer);
		mpu.dmpGetGyro(gg, fifoBuffer);
		mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
		mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
		mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
		
		//Set Mpu Data Fields of Message with Fresh Received Data
		setMpuDataFieldsOfMessage();

		mpuProcessTaskDuration = micros() - mpuProcessStartTime;
	}
}

void setMpuDataFieldsOfMessage()
{
	MsgT01.message.mpuGyroX = gg[0];
	MsgT01.message.mpuGyroY = gg[1];
	MsgT01.message.mpuGyroZ = gg[2];

	MsgT01.message.mpuAccX = aa.x;
	MsgT01.message.mpuAccY = aa.y;
	MsgT01.message.mpuAccZ = aa.z;

	MsgT01.message.mpuAccRealX = aaReal.x;
	MsgT01.message.mpuAccRealY = aaReal.y;
	MsgT01.message.mpuAccRealZ = aaReal.z;

	MsgT01.message.mpuYaw = ypr[0];
	MsgT01.message.mpuPitch = ypr[1];
	MsgT01.message.mpuRoll = ypr[2];

	MsgT01.message.baroTemp = barometerTemp;
	MsgT01.message.baroAlt = barometerAlt;

	MsgT01.message.compassHdg = compassHdg;

	MsgT01.message.gpsData = 7777;

	//Not needed for now, to be implemented if required
	/*
	MsgT01.message.mpuEulerPsi = 0;
	MsgT01.message.mpuEulerTheta = 0;
	MsgT01.message.mpuEulerPhi = 0;
	*/

}

void serialTransmit()
{
	MsgT01.getPacket();
	Serial.write(MsgT01.dataBytes, sizeof(MsgT01.dataBytes));
}

bool initBarometer()
{
	uint32_t initTime = millis();
	// Initialize MS5611 sensor
	// Ultra high resolution: MS5611_ULTRA_HIGH_RES
	// (default) High resolution: MS5611_HIGH_RES
	// Standard: MS5611_STANDARD
	// Low power: MS5611_LOW_POWER
	// Ultra low power: MS5611_ULTRA_LOW_POWER
	while (!barometer.begin(MS5611_ULTRA_HIGH_RES))
	{
		if (millis() - initTime > BAROMETER_INIT_THRESHOLD)
		{
			statusBaro = statusType_InitFail;
			return false;
		}
		delay(200);
	}
	statusBaro = statusType_Normal;

	delay(100);
	return true;
}

void updateBarometerData()
{
	//Barometer Data Processing
	barometer.runProcess();
	barometerTemp = barometer.readTemperature(true);
	barometerPress = barometer.readPressure(true);
	barometerAlt = barometer.getAltitude(barometerPress);
}

bool initCompass()
{

	if (compass.begin())
	{
		// Set measurement range
		compass.setRange(HMC5883L_RANGE_1_3GA);
		// Set measurement mode
		compass.setMeasurementMode(HMC5883L_CONTINOUS);
		// Set data rate
		compass.setDataRate(HMC5883L_DATARATE_15HZ);
		// Set number of samples averaged
		compass.setSamples(HMC5883L_SAMPLES_1);
		// Check settings
		compass.setOffset(COMPASS_OFFSET_X_DEFAULT, COMPASS_OFFSET_Y_DEFAULT);
		statusCompass = statusType_Normal;
		return true;
	}
	else
	{
		statusCompass = statusType_InitFail;
		return false;
	}


}

void updateCompassData()
{
	Vector compassNorm = compass.readNormalize();
	// To calculate heading in degrees. 0 degree indicates North
	compassHdg = atan2(compassNorm.YAxis, compassNorm.XAxis);
}