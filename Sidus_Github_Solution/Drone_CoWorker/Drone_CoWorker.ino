/*
 Name:		Drone_CoWorker.ino
 Created:	12/2/2016 6:37:46 PM
 Author:	SIDUS
*/
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>

//Local include class and files
#include "Local_I2Cdev.h"
#include "Local_MPU6050_6Axis_MotionApps20.h"
#include "Config_CoWorker.h"
#include "cMsgCoWorkerTx.h"
#include "cMsgR01.h"
#include "cMsgT01.h"
#include "cMsgUdpR01.h"
#include "cMsgUdpT01.h"
#include "Local_Agenda.h"
#include "cSerialParse.h"
#include "Local_MS5611.h"
#include "Local_HMC5883L.h"
#include "cUdpInterfaceClass.h"

//Global Class Definitions
Agenda scheduler;
structMsgCoWorkerTx MsgCoWorkerTx;
cMsgT01 MsgT01;
cMsgR01 MsgR01;
cMsgUdpR01 MsgUdpR01;
cMsgUdpT01 MsgUdpT01;
cSerialParse serialParse(sizeof(MsgR01.message), MsgR01.message.startChar1, MsgR01.message.startChar2, MsgR01.message.endChar);
MPU6050 mpu;
MS5611 barometer;
HMC5883L compass;
UdpInterfaceClass myUdp(WIFI_SSID, WIFI_PASS, UDP_PORT, DEFAULT_GROUND_STATION_IP);

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
	
	initVariables();
	initUdp();
	initMPU();
	initBarometer();
	initCompass();

	//Insert all tasks into scheduler
	//scheduler.insert(test_task, 500000);
	scheduler.insert(serialCheck, 15000);
	scheduler.insert(processMpuTask, 5000);
	scheduler.insert(setCoWorkerTxDataFields, 20000);
	scheduler.insert(serialTransmit, 20000);
	scheduler.insert(updateBarometerData, 19000);
	scheduler.insert(updateCompassData, 37000);
	scheduler.insert(udpTransmit, 33000);
	scheduler.insert(udpCheck, 25000);
	scheduler.insert(checkMpuHealth, 1000000);
	scheduler.insert(checkBaroDataReliability, 1000000);
	scheduler.insert(readBatteryVoltage, 770000);

}
// the loop function runs over and over again until power down or reset
void loop() {
	//Just call scheduler update and let it do all the process
	scheduler.update();
}

void test_task()
{
	Serial.println("test task");
}

void initVariables()
{
	statusBaro = statusType_NotInitiated;
	statusMpu = statusType_NotInitiated;
	statusCompass = statusType_NotInitiated;
	statusUdp = statusType_NotInitiated;
	
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
		statusMpu = statusType_Normal;
		Serial.println("MPU6050 connection successful");
	}
	else
	{
		statusMpu = statusType_Fail;
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


		mpuLastDataTime = millis();

	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)

		statusMpu = statusType_Fail;
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
}

void initUdp()
{
	if (myUdp.initConnection(10000, false))
	{
		statusUdp = statusType_Normal;
	}
	else
	{
		statusUdp = statusType_InitFail;
	}
}

void processMpuTask()
{

	//get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();

	//If mpu connection is lost, re initialize mpu, check the code mpuIntStatus statement again!
	if (mpuIntStatus == 0)
	{
		//initMPU();
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

		mpuLastDataTime = millis();

		mpuProcessTaskDuration = micros() - mpuProcessStartTime;
	}
}

void checkMpuHealth()
{
	if (millis() - mpuLastDataTime> MPU_DATATIME_THESHOLD)
	{
		statusMpu = statusType_Fail;
		//try to restart MPU
		initMPU();
	}
	else
	{
		statusMpu = statusType_Normal;
	}
}

void setCoWorkerTxDataFields()
{
	MsgCoWorkerTx.statusMpu = statusMpu;
	MsgCoWorkerTx.statusBaro = statusBaro;
	MsgCoWorkerTx.statusCompass = statusCompass;

	MsgCoWorkerTx.mpuGyroX = gg[0];
	MsgCoWorkerTx.mpuGyroY = gg[1];
	MsgCoWorkerTx.mpuGyroZ = gg[2];

	MsgCoWorkerTx.mpuAccX = aa.x;
	MsgCoWorkerTx.mpuAccY = aa.y;
	MsgCoWorkerTx.mpuAccZ = aa.z;

	MsgCoWorkerTx.mpuAccRealX = aaReal.x;
	MsgCoWorkerTx.mpuAccRealY = aaReal.y;
	MsgCoWorkerTx.mpuAccRealZ = aaReal.z;

	MsgCoWorkerTx.mpuYaw = ypr[0];
	MsgCoWorkerTx.mpuPitch = ypr[1];
	MsgCoWorkerTx.mpuRoll = ypr[2];

	MsgCoWorkerTx.baroTemp = barometerTemp;
	MsgCoWorkerTx.baroAlt = barometerAlt;

	MsgCoWorkerTx.compassHdg = compassHdg;

	MsgCoWorkerTx.batteryVoltageInBits = batteryVoltageInBits;
}

void serialCheck()
{
	int numberofbytes = Serial.available();
	if (numberofbytes > 0)
	{
		//If available number of bytes is less than our buffer size, normal case
		if (numberofbytes <= sizeof(MsgR01.message) * 3)
		{
			unsigned char buffer[sizeof(MsgR01.message) * 3];
			Serial.readBytes(buffer, numberofbytes);
			serialParse.Push(buffer, numberofbytes);
			if (serialParse.getParsedData(MsgR01.dataBytes, sizeof(MsgR01.message)))
			{
				MsgR01.setPacket();
			}
		}
		//Else if buffer overflow, abnormal case
		else
		{
			//Just read it
			unsigned char buffer[sizeof(MsgR01.message) * 3];
			Serial.readBytes(buffer, sizeof(MsgR01.message) * 3);
		}
	}
}

void serialTransmit()
{
	MsgT01.message.coWorkerTxPacket = MsgCoWorkerTx;
	MsgT01.message.udpT01RelayPacket = MsgUdpT01.message;

	MsgT01.getPacket();
	Serial.write(MsgT01.dataBytes, sizeof(MsgT01.dataBytes));

	//Serial.print(millis());
	//Serial.print(",");
	//Serial.println(-MsgT01.message.coWorkerTxPacket.mpuGyroY);
}

void udpTransmit()
{
	MsgUdpR01.message.coWorkerTxPacket = MsgCoWorkerTx;
	MsgUdpR01.message.serialR01RelayPacket = MsgR01.message;

	MsgUdpR01.getPacket();
	myUdp.sendPacket(MsgUdpR01.dataBytes, sizeof(MsgUdpR01.dataBytes));
	//Serial.println("udp messages sent");
	 
}

void udpCheck()
{
	if (myUdp.udp.parsePacket() >= sizeof(MsgUdpT01.dataBytes))
	{
		myUdp.udp.read(MsgUdpT01.dataBytes, sizeof(MsgUdpT01.dataBytes));
		MsgUdpT01.setPacket();
	}

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

void checkBaroDataReliability()
{
	if ((barometerTemp > BARO_TEMP_MIN) && (barometerTemp < BARO_TEMP_MAX) && (barometerAlt > BARO_ALT_MIN) && (barometerAlt < BARO_ALT_MAX))
	{
		statusBaro = statusType_Normal;
	}
	else
	{
		statusBaro = statusType_UnreliableData;
	}
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

void readBatteryVoltage()
{
	batteryVoltageInBits = analogRead(A0);
}