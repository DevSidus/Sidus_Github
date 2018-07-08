/*
Name:		Drone_Flight_Controller.ino
Created:	11/10/2016 8:56:34 PM
Author:	SIDUS
Description: This is the main code for Drone_Flight_Controller Project
*/

//Global Include Files
#include <WiFi.h>
#include <esp32-hal-ledc.h>
#include <Wire.h>
#include <ArduinoOTA.h>
#include <../../tools/sdk/include/driver/driver/rmt.h>
#include <../../c_library_v2/common/mavlink.h>

#include "FS.h"
#include "SD.h"
#include "SPI.h"

using namespace std;
//Local Include Files
#include "kalmanFilter.h"
#include "kalmanFilter_initialize.h"
#include "cDataFilter.h"
#include "cBuzzerMelody.h"
#include "Local_I2Cdev.h"
#include "Local_MPU6050_6Axis_MotionApps20.h"
#include "Local_HMC5883L.h"
#include "Local_PID_v1.h"
#include "PID_YawAngle.h"
#include "LocalTinyGPS++.h"
#include "uBloxGPS.h"
#include "Config.h"

#ifdef BAROMETER_MS5611
#include "Local_MS5611.h"
#endif
#ifdef BAROMETER_BMP180
#include "Local_SFE_BMP180.h"
#endif
#ifdef BAROMETER_BMP280
#include "Local_BMP280.h"
#endif

#include "LocalLidarLite.h"

#include "cMsgUdpR01.h"
#include "cMsgUdpT01.h"
#include "cMsgUdpRAndroid.h"
#include "cMsgUdpTAndroid.h"

#include "cRxFilter.h"

//Global Class Definitions
MPU6050 mpu;

#ifdef BAROMETER_MS5611
MS5611 barometer;
#endif 
#ifdef BAROMETER_BMP180
SFE_BMP180 barometer;
#endif
#ifdef BAROMETER_BMP280
BMP280 barometer;
#endif

HMC5883L compass;
LIDARLite lidar;


WiFiUDP udp;

cMsgUdpR01 MsgUdpR01;
cMsgUdpT01 MsgUdpT01;
cMsgUdpRAndroid MsgUdpRAndroid;
cMsgUdpTAndroid MsgUdpTAndroid;

PID pidRatePitch(&pidVars.ratePitch.sensedVal, &pidVars.ratePitch.output, &pidVars.ratePitch.setpoint, &pidVars.ratePitch.sensedValDiff, &pidVars.ratePitch.setpointDiff);
PID pidAnglePitch(&pidVars.anglePitch.sensedVal, &pidVars.anglePitch.output, &pidVars.anglePitch.setpoint, &pidVars.anglePitch.sensedValDiff, &pidVars.anglePitch.setpointDiff);
PID pidRateRoll(&pidVars.rateRoll.sensedVal, &pidVars.rateRoll.output, &pidVars.rateRoll.setpoint, &pidVars.rateRoll.sensedValDiff, &pidVars.rateRoll.setpointDiff);
PID pidAngleRoll(&pidVars.angleRoll.sensedVal, &pidVars.angleRoll.output, &pidVars.angleRoll.setpoint, &pidVars.angleRoll.sensedValDiff, &pidVars.angleRoll.setpointDiff);
PID pidRateYaw(&pidVars.rateYaw.sensedVal, &pidVars.rateYaw.output, &pidVars.rateYaw.setpoint, &pidVars.rateYaw.sensedValDiff, &pidVars.rateYaw.setpointDiff);
PID_YawAngle pidAngleYaw(&pidVars.angleYaw.sensedVal, &pidVars.angleYaw.output, &pidVars.angleYaw.setpoint, &pidVars.angleYaw.sensedValDiff);

PID pidPosAlt(&pidVars.posAlt.sensedVal, &pidVars.posAlt.output, &pidVars.posAlt.setpoint, &pidVars.posAlt.sensedValDiff, &pidVars.posAlt.setpointDiff);
PID pidVelAlt(&pidVars.velAlt.sensedVal, &pidVars.velAlt.output, &pidVars.velAlt.setpoint, &pidVars.velAlt.sensedValDiff, &pidVars.velAlt.setpointDiff);
PID pidAccAlt(&pidVars.accAlt.sensedVal, &pidVars.accAlt.output, &pidVars.accAlt.setpoint, &pidVars.accAlt.sensedValDiff, &pidVars.accAlt.setpointDiff);

PID pidPosX(&pidVars.posX.sensedVal, &pidVars.posX.output, &pidVars.posX.setpoint, &pidVars.posX.sensedValDiff, &pidVars.posX.setpointDiff);
PID pidPosY(&pidVars.posY.sensedVal, &pidVars.posY.output, &pidVars.posY.setpoint, &pidVars.posY.sensedValDiff, &pidVars.posX.setpointDiff);

PID pidVelX(&pidVars.velX.sensedVal, &pidVars.velX.output, &pidVars.velX.setpoint, &pidVars.velX.sensedValDiff, &pidVars.velX.setpointDiff);
PID pidVelY(&pidVars.velY.sensedVal, &pidVars.velY.output, &pidVars.velY.setpoint, &pidVars.velY.sensedValDiff, &pidVars.velY.setpointDiff);

PID pidAccX(&pidVars.accX.sensedVal, &pidVars.accX.output, &pidVars.accX.setpoint, &pidVars.accX.sensedValDiff, &pidVars.accX.setpointDiff);
PID pidAccY(&pidVars.accY.sensedVal, &pidVars.accY.output, &pidVars.accY.setpoint, &pidVars.accY.sensedValDiff, &pidVars.accY.setpointDiff);

cBuzzerMelody buzzer(PIN_BUZZER, BUZZER_PWM_CHANNEL);

cRxFilter filterRxThr(RX_MAX_PULSE_WIDTH), filterRxPitch(RX_MAX_PULSE_WIDTH), filterRxRoll(RX_MAX_PULSE_WIDTH), filterRxYaw(RX_MAX_PULSE_WIDTH);

cRxFilter filterRx5thCh(RX_MAX_PULSE_WIDTH), filterRx6thCh(RX_MAX_PULSE_WIDTH);

HardwareSerial SerialGps(2);

// The TinyGPS++ object
TinyGPSPlus gps;
uBloxGps gpsUbx;
gpsData uBloxData;

// Mavlink Definitions
mavlink_system_t mavlink_system;
mavlink_message_t mavlinkMessage;
uint8_t mavlinkBuffer[MAVLINK_MAX_PACKET_LEN];
uint16_t mavlinkMessageLength;

File sd_logfile;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(SERIAL_COM_SPEED);
	Serial.println("");
	Serial.println("Serial started");
	
	mavlink_system.sysid = 255;                   ///< ID 255 for this airplane
	mavlink_system.compid = MAV_COMP_ID_ALL; // MAV_COMP_ID_IMU;     ///< The component sending the message is the IMU, it could be also a Linux process

	connectToWiFi();

	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_SDCARD_CS, OUTPUT);
	pinMode(PIN_BATTERY, INPUT);
	pinMode(PIN_RX_ROLL, INPUT);
	pinMode(PIN_RX_PITCH, INPUT);
	pinMode(PIN_RX_THR, INPUT);
	pinMode(PIN_RX_YAW, INPUT);
	pinMode(PIN_RX_5TH_CHAN, INPUT);
	pinMode(PIN_RX_6TH_CHAN, INPUT);
	pinMode(PIN_BUZZER, OUTPUT);
	pinMode(PIN_MPU_POWER_ON, OUTPUT);
	pinMode(PIN_ULTSENS_TRIG, OUTPUT);
	pinMode(PIN_ULTSENS_ECHO, INPUT);
	pinMode(PIN_MCU_SCL, OUTPUT);
	digitalWrite(PIN_MPU_POWER_ON, LOW);
	digitalWrite(PIN_ULTSENS_TRIG, HIGH);

	for (int i = 0; i < 40; i++)
	{
		digitalWrite(PIN_MCU_SCL, LOW);
		delay(1);
		digitalWrite(PIN_MCU_SCL, HIGH);
		delay(1);
	}
	delay(100);
	


	Wire.begin(PIN_MCU_SDA, PIN_MCU_SCL);
	Wire.setClock(420000L);
		

	//Processor 0 Tasks
	xTaskCreatePinnedToCore(task_test, "task_test", 1024, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_0, "task_rx_0", 1200, NULL, 2, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_1, "task_rx_1", 1200, NULL, 3, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_2, "task_rx_2", 1200, NULL, 4, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_3, "task_rx_3", 1200, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_4, "task_rx_4", 1200, NULL, 6, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_5, "task_rx_5", 1200, NULL, 7, NULL, 0);
	xTaskCreatePinnedToCore(task_UDP, "task_UDP", 3072, NULL, 8, NULL, 0);
	xTaskCreatePinnedToCore(task_mapCmd, "task_mapCmd", 1024, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_chkMode, "task_chkMode", 1024, NULL, 11, NULL, 0);
	xTaskCreatePinnedToCore(task_ADC, "task_ADC", 1024, NULL, 12, NULL, 0);
	xTaskCreatePinnedToCore(task_melody, "task_melody", 1024, NULL, 13, NULL, 0);
	xTaskCreatePinnedToCore(task_altitude_kalman, "task_altitude_kalman", 2048, NULL, 15, NULL, 0);
	xTaskCreatePinnedToCore(task_position_kalman, "task_position_kalman", 3072, NULL, 16, NULL, 0);
	xTaskCreatePinnedToCore(task_compass_kalman, "task_compass_kalman", 2048, NULL, 17, NULL, 0);
	//xTaskCreatePinnedToCore(task_OTA, "task_OTA", 3072, NULL, 20, NULL, 0);
	//xTaskCreatePinnedToCore(task_IoT, "task_IoT", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_gps, "task_gps", 2048, NULL, 10, NULL, 0);

	//Processor 1 Tasks
	xTaskCreatePinnedToCore(task_mpu, "task_mpu", 2400, NULL, 20, NULL, 1);
	xTaskCreatePinnedToCore(task_PID, "task_PID", 3072, NULL, 15, NULL, 1);
	xTaskCreatePinnedToCore(task_Motor, "task_Motor", 1536, NULL, 18, NULL, 1);
	//xTaskCreatePinnedToCore(task_lidar, "task_lidar", 1200, NULL, 0, NULL, 1);
#ifndef USE_SD_CARD
	xTaskCreatePinnedToCore(task_ultrasonic, "task_ultrasonic", 1536, NULL, 10, NULL, 1);
#endif // USE_SD_CARD



	//esp_event_loop_init((system_event_cb_t*)&task_UDPhandle, NULL);
}

void appendFile(fs::FS &fs, const char * path, const uint8_t * message, size_t size) {

	if (!sd_logfile)	return;

	if (sd_logfile.write(message, size))	appendPacketCounter++;
	//else	//Serial.println("Append failed");

	if (appendPacketCounter % 60 == 0)   //at every # of packet append, close and reopen file
	{
		sd_logfile.close();
		sd_logfile = fs.open(sdcard_filepath, FILE_APPEND);
	}
}

void createFile(fs::FS &fs) {

	for (int i = 0; i < 9999; i++)
	{
		sd_logfile = fs.open("/" + String(i) + ".bin");
		if (!sd_logfile)
		{
			//Serial.print("i:");
			//Serial.println(i);
			sd_logfile = fs.open("/" + String(i) + ".bin", FILE_WRITE);
			if (!sd_logfile)
			{
				Serial.println("Failed to open file for writing");
				return;
			}
			else
			{

				sd_logfile.close();

				sdcard_filepath = "/" + String(i) + ".bin";
				Serial.println("File Created: " + sdcard_filepath);

				sd_logfile = fs.open(sdcard_filepath, FILE_APPEND);

				if (!sd_logfile)
				{
					Serial.println("File Append Could not Opened!");
				}
				else
				{
					Serial.println("File Append Opened");
				}
			}
			return;
		}
		else
		{
			sd_logfile.close();
		}
	}

}
// the loop function runs over and over again until power down or reset
void loop() {
	vTaskSuspend(NULL);
}

void task_test(void * parameter)
{
	while (true)
	{
		processTest();
		//Serial.println(portTICK_PERIOD_MS);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));	   
	}
	vTaskDelete(NULL);
}

void task_mpu(void * parameter)
{
	int i2c_task_counter = 0;
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 5;
	// Initialise the xLastWakeTime variable with the current time.
	xLastWakeTime = xTaskGetTickCount();
	statusMpu = statusType_NotInitiated;

	initMPU();
	initCompass();
	initBarometer();


	while (true)
	{

		processMpu();

		if (i2c_task_counter % 6 == 0)
			processCompass();

		if (i2c_task_counter % 10 == 0)
			processBarometer();

		if(i2c_task_counter % 100 == 0)
			checkMpuGSHealth();

		i2c_task_counter++;

		if (i2c_task_counter == 6000)
			i2c_task_counter = 0;

		vTaskDelayUntil(&xLastWakeTime, xFrequency);
	}
	vTaskDelete(NULL);
	return;
}

void task_ultrasonic(void * parameter)
{
	double pulseDuration = 0.0;
	double speedOfSound = 340.29; // m/s
	double maxRange = 4.0; // m
	double ultrasonicDistanceOld = 0.0; // old sensor measurement, to deal with burst values

	// If there is an offset between ground and sensor, substract this offset
	double ultrasonicDistanceOffset = 0.0; // m

	while (true)
	{
		// Clears the trigPin
		digitalWrite(PIN_ULTSENS_TRIG, LOW);
		delayMicroseconds(2);
		// Sets the trigPin on HIGH state for 10 micro seconds
		digitalWrite(PIN_ULTSENS_TRIG, HIGH);
		delayMicroseconds(10);
		digitalWrite(PIN_ULTSENS_TRIG, LOW);

		// Reads the echoPin, returns the sound wave travel time in microseconds
		pulseDuration = pulseIn(PIN_ULTSENS_ECHO, HIGH);
		
		// Calculating the distance
		ultrasonicDistance = pulseDuration * speedOfSound * 1e-6 / 2 - ultrasonicDistanceOffset; // in m

		if (ultrasonicDistance > maxRange) {
			ultrasonicDistance = ultrasonicDistanceOld;
		}

		// Low Pass filter
		// Calculate Filter Output		
		ultrasonicDistanceFiltered = filtObjUltrasonicDist.filter(ultrasonicDistance, 0);

		ultrasonicDistanceOld = ultrasonicDistance;

		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(50);
	}
	vTaskDelete(NULL);
	return;
}

void task_gps(void * parameter)
{

	// Start with GPS's default configuration
	SerialGps.begin(SERIAL_DEFAULT_GPS_SPEED);
	delay(200);

	// Change GPS Port configuration
	SerialGps.write(ubxPortConfigPacketPart1,28);
	delay(200);
	//SerialGps.write(ubxPortConfigPacketPart2, 9);
	//delay(1000);
	
	SerialGps.begin(SERIAL_GPS_SPEED);
	delay(200);

	// change frequency to 10 Hz
	SerialGps.write(ubxRateConfigPacketPart1, 14);
	delay(200);
	//SerialGps.write(ubxRateConfigPacketPart2, 8);
	//delay(1000);

	// Enable NAV-PVT messages
	SerialGps.write(ubxMessageConfigPacketPart1,16);
	delay(200);
	//SerialGps.write(ubxMessageConfigPacketPart2, 10);
	//delay(1000);

	uint8_t val;

	while (true)
	{

		while (SerialGps.available() > 0)
		{
			val = SerialGps.read();

			#ifndef UBOX
				gps.encode(val);

				if (gps.location.isUpdated())
				{
					qcGPS.lat = gps.location.lat();
					qcGPS.lon = gps.location.lng();
				}
				if (gps.altitude.isUpdated())
				{
					qcGPS.alt = gps.altitude.meters();
				}
				if (gps.hdop.isUpdated())
				{
					qcGPS.hdop = gps.hdop.value();
				}
				if (gps.speed.isUpdated())
				{
					qcGPS.sog = gps.speed.mps();
				}
				if (gps.course.isUpdated())
				{
					qcGPS.cog = gps.course.deg();
				}
				if (gps.speed.isUpdated() && gps.course.isUpdated())
				{
					qcGPS.nedVelocity.N = qcGPS.sog * cos(qcGPS.cog * M_PI / 180);
					qcGPS.nedVelocity.E = qcGPS.sog * sin(qcGPS.cog * M_PI / 180);
		}
			
			#endif // !UBOX

			if (gpsUbx.parse(val)) {
				gpsUbx.read(&uBloxData);
				qcGPS.lat = uBloxData.lat;
				qcGPS.lon = uBloxData.lon;
				qcGPS.alt = uBloxData.hMSL;
				qcGPS.posAccuracy = uBloxData.hAcc;
				qcGPS.nedVelocity.N = uBloxData.velN;
				qcGPS.nedVelocity.E = uBloxData.velE;
				qcGPS.nedVelocity.D = uBloxData.velD;
				qcGPS.velAccuracy = uBloxData.sAcc;
				qcGPS.gpsFixType = uBloxData.fixType;
			}

		}

		#ifndef UBOX
			if (gps.location.age() > GPS_UPDATE_THRESHOLD_TIME || !gps.location.isValid())
			{
				qcGPS.gpsIsFix = false;
				gpsPositionAvailable = false;
				gpsVelocityAvailable = false;
			}
			else
			{
				qcGPS.gpsIsFix = true;
				gpsVelocityAvailable = true;
			}
		#else
			if ((qcGPS.gpsFixType == fix3D) || (qcGPS.gpsFixType == fix2D)) // GPS should be used only at 2D or 3D Fix Mode
			{
				qcGPS.gpsIsFix = true;
				gpsVelocityAvailable = true; // GPS velocity can be used at 2D or 3D Fix Mode
			}
			else
			{
				qcGPS.gpsIsFix = false;
				gpsPositionAvailable = false;
				gpsVelocityAvailable = false;
			}
		#endif // !UBOX
		
		if (qcGPS.gpsIsFix)
		{

			// Calculate World X/Y Position from GPS measurements
			calculateGeodetic2Ecef(qcGPS.lat, qcGPS.lon, qcGPS.alt, &qcGPS.ecefCoordinate.x, &qcGPS.ecefCoordinate.y, &qcGPS.ecefCoordinate.z);

			if (setHomePoint && !homePointSelected)
			{

				homePoint.ecefCoordinate.x = qcGPS.ecefCoordinate.x;
				homePoint.ecefCoordinate.y = qcGPS.ecefCoordinate.y;
				homePoint.ecefCoordinate.z = qcGPS.ecefCoordinate.z;
				homePoint.lat = qcGPS.lat;
				homePoint.lon = qcGPS.lon;
				homePoint.alt = qcGPS.alt;

				homePointSelected = true;
			}

			if (homePointSelected)
			{
				calculateEcef2Ned(qcGPS.ecefCoordinate.x, qcGPS.ecefCoordinate.y, qcGPS.ecefCoordinate.z, homePoint.ecefCoordinate.x, homePoint.ecefCoordinate.y, homePoint.ecefCoordinate.z, homePoint.lat, homePoint.lon, &qcGPS.nedCoordinate.N, &qcGPS.nedCoordinate.E, &qcGPS.nedCoordinate.D);

				gpsPositionAvailable = true; // GPS velocity can be used at 2D or 3D Fix Mode and if home point is selected

			}
		}

		// Serial.println(uxTaskGetStackHighWaterMark(NULL));

		delay(20);
	}
	vTaskDelete(NULL);
	return;
}

void displayInfo()
{
	Serial.print(F("Location: "));
	if (gps.location.isValid())
	{
		Serial.print(gps.location.lat(), 6);
		Serial.print(F(","));
		Serial.print(gps.location.lng(), 6);
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F("  Date/Time: "));
	if (gps.date.isValid())
	{
		Serial.print(gps.date.month());
		Serial.print(F("/"));
		Serial.print(gps.date.day());
		Serial.print(F("/"));
		Serial.print(gps.date.year());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.print(F(" "));
	if (gps.time.isValid())
	{
		if (gps.time.hour() < 10) Serial.print(F("0"));
		Serial.print(gps.time.hour());
		Serial.print(F(":"));
		if (gps.time.minute() < 10) Serial.print(F("0"));
		Serial.print(gps.time.minute());
		Serial.print(F(":"));
		if (gps.time.second() < 10) Serial.print(F("0"));
		Serial.print(gps.time.second());
		Serial.print(F("."));
		if (gps.time.centisecond() < 10) Serial.print(F("0"));
		Serial.print(gps.time.centisecond());
	}
	else
	{
		Serial.print(F("INVALID"));
	}

	Serial.println();
}

void task_rx_0(void * parameter)
{
	gpio_set_pull_mode(gpio_num_t(PIN_RX_ROLL), GPIO_PULLDOWN_ONLY);

	delay(200);
	//--------RMT CONFIG RADIO ROLL CHANNEL-----------------//
	rmt_config_t rmt_rx_ch0;
	rmt_rx_ch0.channel = RMT_CHANNEL_0;
	rmt_rx_ch0.gpio_num = gpio_num_t(PIN_RX_ROLL);
	rmt_rx_ch0.clk_div = RMT_CLK_DIV;
	rmt_rx_ch0.mem_block_num = 1;
	rmt_rx_ch0.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch0.rx_config.filter_en = true;
	rmt_rx_ch0.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch0.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch0);
	rmt_driver_install(rmt_rx_ch0.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch0 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch0.channel, &rb_ch0);
	rmt_rx_start(rmt_rx_ch0.channel, true);
	//--------RMT CONFIG RADIO ROLL CHANNEL-----------------//

	size_t rx_size = 0;
	rmt_item32_t* item_ch0 = NULL;
	while (true) 
	{
		//------------RMT GET RADIO ROLL PULSE DURATION-----------//
		rx_size = 0;
		
		item_ch0 = (rmt_item32_t*)xRingbufferReceive(rb_ch0, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch0)
		{
			//Get the duration values from item..
			//Serial.print("Roll: ");
			//Serial.println(item_ch0->duration0);

			dutyCycle_Roll = item_ch0->duration0;

			vRingbufferReturnItem(rb_ch0, (void*)item_ch0);
		}
		//------------RMT GET RADIO ROLL PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_rx_1(void * parameter)
{

	gpio_set_pull_mode(gpio_num_t(PIN_RX_PITCH), GPIO_PULLDOWN_ONLY);


	delay(300);
	//--------RMT CONFIG RADIO PITCH CHANNEL-----------------//
	rmt_config_t rmt_rx_ch1;
	rmt_rx_ch1.channel = RMT_CHANNEL_1;
	rmt_rx_ch1.gpio_num = gpio_num_t(PIN_RX_PITCH);
	rmt_rx_ch1.clk_div = RMT_CLK_DIV;
	rmt_rx_ch1.mem_block_num = 1;
	rmt_rx_ch1.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch1.rx_config.filter_en = true;
	rmt_rx_ch1.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch1.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch1);
	rmt_driver_install(rmt_rx_ch1.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch1 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch1.channel, &rb_ch1);
	rmt_rx_start(rmt_rx_ch1.channel, true);
	//--------RMT CONFIG RADIO PITCH CHANNEL-----------------//
	size_t rx_size = 0;
	rmt_item32_t* item_ch1 = NULL;
	while (true)
	{

		//------------RMT GET RADIO PITCH PULSE DURATION-----------//
		rx_size = 0;
		item_ch1 = (rmt_item32_t*)xRingbufferReceive(rb_ch1, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch1)
		{
			//Get the duration values from item..
			//Serial.print("Pitch: ");
			//Serial.println(item_ch1->duration0);

			dutyCycle_Pitch = item_ch1->duration0;

			vRingbufferReturnItem(rb_ch1, (void*)item_ch1);
		}
		//------------RMT GET RADIO PITCH PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_rx_2(void * parameter)
{

	gpio_set_pull_mode(gpio_num_t(PIN_RX_THR), GPIO_PULLDOWN_ONLY);

	delay(400);
	//--------RMT CONFIG RADIO THR CHANNEL-----------------//
	rmt_config_t rmt_rx_ch2;
	rmt_rx_ch2.channel = RMT_CHANNEL_2;
	rmt_rx_ch2.gpio_num = gpio_num_t(PIN_RX_THR);
	rmt_rx_ch2.clk_div = RMT_CLK_DIV;
	rmt_rx_ch2.mem_block_num = 1;
	rmt_rx_ch2.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch2.rx_config.filter_en = true;
	rmt_rx_ch2.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch2.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch2);
	rmt_driver_install(rmt_rx_ch2.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch2 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch2.channel, &rb_ch2);
	rmt_rx_start(rmt_rx_ch2.channel, true);
	//--------RMT CONFIG RADIO THR CHANNEL-----------------//

	size_t rx_size = 0;
	rmt_item32_t* item_ch2 = NULL;
	while (true)
	{
		//------------RMT GET RADIO THR PULSE DURATION-----------//
		rx_size = 0;
		item_ch2 = (rmt_item32_t*)xRingbufferReceive(rb_ch2, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch2)
		{
			//Get the duration values from item..

			//Get the duration values from item..
			//Serial.print("Thr: ");
			//Serial.println(item_ch2->duration0);
			dutyCycle_Thr = item_ch2->duration0;
			rxLastDataTime = millis();  //we need to define this for each isr in order to fully get status of rx
			statusRx = statusType_Normal;
			vRingbufferReturnItem(rb_ch2, (void*)item_ch2);
		}
		else
		{
			if (millis() - rxLastDataTime > RX_DATATIME_THRESHOLD)
			{
				statusRx = statusType_Fail;
			}
		}
		//------------RMT GET RADIO THR PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_rx_3(void * parameter)
{
	gpio_set_pull_mode(gpio_num_t(PIN_RX_YAW), GPIO_PULLDOWN_ONLY);

	delay(500);
	//--------RMT CONFIG RADIO YAW CHANNEL-----------------//
	rmt_config_t rmt_rx_ch3;
	rmt_rx_ch3.channel = RMT_CHANNEL_3;
	rmt_rx_ch3.gpio_num = gpio_num_t(PIN_RX_YAW);
	rmt_rx_ch3.clk_div = RMT_CLK_DIV;
	rmt_rx_ch3.mem_block_num = 1;
	rmt_rx_ch3.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch3.rx_config.filter_en = true;
	rmt_rx_ch3.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch3.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch3);
	rmt_driver_install(rmt_rx_ch3.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch3 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch3.channel, &rb_ch3);
	rmt_rx_start(rmt_rx_ch3.channel, true);
	//--------RMT CONFIG RADIO YAW CHANNEL-----------------//

	size_t rx_size = 0;
	rmt_item32_t* item_ch3 = NULL;
	while (true)
	{

		//------------RMT GET RADIO YAW PULSE DURATION-----------//
		rx_size = 0;
		item_ch3 = (rmt_item32_t*)xRingbufferReceive(rb_ch3, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch3)
		{
			//Get the duration values from item..
			//Serial.print("Yaw: ");
			//Serial.println(item_ch3->duration0);
			dutyCycle_Yaw = item_ch3->duration0;
			vRingbufferReturnItem(rb_ch3, (void*)item_ch3);
		}
		//------------RMT GET RADIO YAW PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_rx_4(void * parameter)
{
	gpio_set_pull_mode(gpio_num_t(PIN_RX_5TH_CHAN), GPIO_PULLDOWN_ONLY);

	delay(600);
	//--------RMT CONFIG RADIO 5TH_CHAN CHANNEL-----------------//
	rmt_config_t rmt_rx_ch4;
	rmt_rx_ch4.channel = RMT_CHANNEL_4;
	rmt_rx_ch4.gpio_num = gpio_num_t(PIN_RX_5TH_CHAN);
	rmt_rx_ch4.clk_div = RMT_CLK_DIV;
	rmt_rx_ch4.mem_block_num = 1;
	rmt_rx_ch4.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch4.rx_config.filter_en = true;
	rmt_rx_ch4.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch4.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch4);
	rmt_driver_install(rmt_rx_ch4.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch4 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch4.channel, &rb_ch4);
	rmt_rx_start(rmt_rx_ch4.channel, true);
	//--------RMT CONFIG RADIO 5TH_CHAN CHANNEL-----------------//

	size_t rx_size = 0;
	rmt_item32_t* item_ch4 = NULL;
	while (true)
	{

		//------------RMT GET RADIO 5TH CHAN PULSE DURATION-----------//
		rx_size = 0;
		item_ch4 = (rmt_item32_t*)xRingbufferReceive(rb_ch4, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch4)
		{
			//Get the duration values from item..
			//Serial.print("5th: ");
			//Serial.println(item_ch4->duration0);
			dutyCycle_Rx5thCh = item_ch4->duration0;
			vRingbufferReturnItem(rb_ch4, (void*)item_ch4);
		}
		//------------RMT GET RADIO 5TH CHAN PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_rx_5(void * parameter)
{
	gpio_set_pull_mode(gpio_num_t(PIN_RX_6TH_CHAN), GPIO_PULLDOWN_ONLY);

	delay(700);
	//--------RMT CONFIG RADIO 6TH_CHAN CHANNEL-----------------//
	rmt_config_t rmt_rx_ch5;
	rmt_rx_ch5.channel = RMT_CHANNEL_5;
	rmt_rx_ch5.gpio_num = gpio_num_t(PIN_RX_6TH_CHAN);
	rmt_rx_ch5.clk_div = RMT_CLK_DIV;
	rmt_rx_ch5.mem_block_num = 1;
	rmt_rx_ch5.rmt_mode = RMT_MODE_RX;
	rmt_rx_ch5.rx_config.filter_en = true;
	rmt_rx_ch5.rx_config.filter_ticks_thresh = RMT_FILTER_TICK_THRESHOLD;
	rmt_rx_ch5.rx_config.idle_threshold = RMT_IDLE_THRESHOLD;
	rmt_config(&rmt_rx_ch5);
	rmt_driver_install(rmt_rx_ch5.channel, RMT_RX_BUFFER_SIZE, 0);

	RingbufHandle_t rb_ch5 = NULL;
	rmt_get_ringbuf_handle(rmt_rx_ch5.channel, &rb_ch5);
	rmt_rx_start(rmt_rx_ch5.channel, true);
	//--------RMT CONFIG RADIO 6TH_CHAN CHANNEL-----------------//

	size_t rx_size = 0;
	rmt_item32_t* item_ch5 = NULL;

	while (true)
	{
		//------------RMT GET RADIO 6TH CHAN PULSE DURATION-----------//
		rx_size = 0;
		item_ch5 = (rmt_item32_t*)xRingbufferReceive(rb_ch5, &rx_size, RMT_RX_WAIT_TICKS);
		if (item_ch5)
		{
			//Get the duration values from item..
			//Serial.print("6th: ");
			//Serial.println(item_ch5->duration0);
			dutyCycle_Rx6thCh = item_ch5->duration0;
			vRingbufferReturnItem(rb_ch5, (void*)item_ch5);
		}
		//------------RMT GET RADIO 6TH CHAN PULSE DURATION-----------//

		delay(10);
	}
	vTaskDelete(NULL);
}

void task_mapCmd(void * parameter)
{
	
	while (true)
	{
		processMapRxDCtoCmd();
		delay(19);
	}
	vTaskDelete(NULL);
}

void task_chkMode(void * parameter)
{

	modeQuad = modeQuadSAFE;
	autoModeStatus = autoModeOFF;
	statusRx = statusType_NotInitiated;

	while (true)
	{
		processCheckMode();
		handleAutoModeCommands();
		delay(200);
	}
	vTaskDelete(NULL);
}

void task_PID(void * parameter)
{
	initPIDvariables();
	initPIDtuning();

	while (true)
	{
		processPID();
		delay(9);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
	}
	vTaskDelete(NULL);
}

void task_Motor(void * parameter)
{
	setupMotorPins();

	while (true)
	{
		processRunMotors();
		delay(9);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
	}
	vTaskDelete(NULL);
}

void task_UDP(void * parameter)
{
	int udp_task_counter = 0;


	#ifdef USE_SD_CARD
		delay(500);
		initSDcard();
		delay(100);
		createFile(SD);
	#endif // USE_SD_CARD

	#ifdef MAVLINK_PROTOCOL
		qcMavlink.type = MAV_TYPE_QUADROTOR;
		qcMavlink.autopilot = MAV_AUTOPILOT_GENERIC;

		qcMavlink.base_mode = MAV_MODE_FLAG_AUTO_ENABLED;
		qcMavlink.custom_mode = 0; ///< Custom mode, can be defined by user/adopter
		qcMavlink.system_status = MAV_STATE_STANDBY;  ///< System ready for flight
	#endif // MAVLINK_PROTOCOL

	int mavlinkPacketSize;
	char mavlinkPacketBuffer[MAVLINK_MAX_PACKET_LEN];

	while (true)
	{
		prepareUDPmessages();

		#ifdef SIDUS_ANDROID_PROTOCOL
			prepareAndroidUDPmessages();
		#endif // SIDUS_ANDROID_PROTOCOL

		
		if (wifi_connected)
		{
			if (udp_connected)
			{
				if (udp.beginPacket(DEFAULT_GROUND_STATION_IP, UDP_PORT) != 1)
				{
					//Serial.println("Could not begin UDP packet");
				}
				else
				{
					#ifdef SIDUS_PROTOCOL
						#ifdef SIDUS_ANDROID_PROTOCOL
							udp.write(MsgUdpRAndroid.dataBytes, sizeof(MsgUdpRAndroid.dataBytes));
						#else // SIDUS_PROTOCOL
							udp.write(MsgUdpR01.dataBytes, sizeof(MsgUdpR01.dataBytes));
						#endif // SIDUS_ANDROID_PROTOCOL

					#else // MAVLINK_PROTOCOL	
						
						mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &mavlinkMessage,
							qcMavlink.custom_mode, qcMavlink.type, qcMavlink.autopilot, qcMavlink.base_mode, qcMavlink.system_status);
						mavlinkMessageLength = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessage);
						udp.write(mavlinkBuffer, mavlinkMessageLength);

						/*mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &mavlinkMessage,
							MAV_SYS_STATUS_SENSOR_3D_GYRO, MAV_SYS_STATUS_SENSOR_3D_GYRO, MAV_SYS_STATUS_SENSOR_3D_GYRO, 0, batteryVoltageInVolts, -1, 100, 0, 0, 0, 0, 0, 0);
						mavlinkMessageLength = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessage);
						udp.write(mavlinkBuffer, mavlinkMessageLength);

						mavlink_msg_autopilot_version_pack(mavlink_system.sysid, mavlink_system.compid, &mavlinkMessage,
							MAV_PROTOCOL_CAPABILITY_MAVLINK2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
						mavlinkMessageLength = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessage);
						udp.write(mavlinkBuffer, mavlinkMessageLength);*/
						
						mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &mavlinkMessage,
							millis(), qc.euler.phi, qc.euler.theta, compassHdgEstimated *  M_PI / 180, qc.eulerRate.phi, qc.eulerRate.theta, qc.eulerRate.psi);
						mavlinkMessageLength = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessage);
						udp.write(mavlinkBuffer, mavlinkMessageLength);

						mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &mavlinkMessage,
							micros(), qcGPS.gpsFixType, qcGPS.lat*1e7, qcGPS.lon*1e7, qcGPS.alt*1e3, 0, 0, qcGPS.nedVelocity.E, 0, qcGPS.satellites_visible, 0, qcGPS.posAccuracy, 0, qcGPS.velAccuracy, 0);
						mavlinkMessageLength = mavlink_msg_to_send_buffer(mavlinkBuffer, &mavlinkMessage);
						udp.write(mavlinkBuffer, mavlinkMessageLength);

					#endif // SIDUS_PROTOCOL
						

					if (udp.endPacket() != 1)
					{
						//Serial.println("UDP Packet could not send");
						//connectUdp();
					}
					else
					{
						//Serial.println(millis());
					}
				}
				//Serial.print("X");
			}			
		}

		#ifdef USE_SD_CARD
				appendFile(SD, sdcard_filepath.c_str(), MsgUdpR01.dataBytes, sizeof(MsgUdpR01.message));
		#endif // USE_SD_CARD

		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		//Serial.println("t_ok");


		//UDP RX Operations
		if (wifi_connected && udp_connected && (udp_task_counter % 15 == 0))
		{

			#ifdef SIDUS_PROTOCOL
			#ifdef SIDUS_ANDROID_PROTOCOL
			if (udp.parsePacket() >= sizeof(MsgUdpTAndroid.dataBytes))
			{
				udp.read(MsgUdpTAndroid.dataBytes, sizeof(MsgUdpTAndroid.dataBytes));
				MsgUdpTAndroid.setPacket();

				updateAndroidUdpMsgVars();

				// Serial.print(setHomePoint);

				//Serial.println("udp message received");
				//udpLastMessageTime = millis();
			}
			#else // SIDUS_PROTOCOL
			if (udp.parsePacket() >= sizeof(MsgUdpT01.dataBytes))
			{
				udp.read(MsgUdpT01.dataBytes, sizeof(MsgUdpT01.dataBytes));
				MsgUdpT01.setPacket();

				updateUdpMsgVars();

				//Serial.println("udp message received");
				//udpLastMessageTime = millis();
			}
			#endif // SIDUS_ANDROID_PROTOCOL

			#else // MAVLINK_PROTOCOL
			mavlinkPacketSize = udp.parsePacket();

			if (mavlinkPacketSize)
			{
				udp.read(mavlinkPacketBuffer, MAVLINK_MAX_PACKET_LEN);
				// Serial.println(mavlinkPacketBuffer);
			}

			// This code part will be added when Mavlink Receive Channel is available

			#endif // SIDUS_PROTOCOL

			//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		}

		udp_task_counter++;

		if (udp_task_counter == 6000)
			udp_task_counter = 0;

		delay(10);

	}


	vTaskDelete(NULL);
}

void task_OTA(void * parameter)
{
	initOTA();

	while (true)
	{
		if (wifi_connected)
		{
			ArduinoOTA.handle();
		}
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(200);
	}
	vTaskDelete(NULL);
}

void task_ADC(void * parameter)
{
	//adcAttachPin(PIN_BATTERY);
	//adcStart(PIN_BATTERY);

	while (true)
	{
		//adcEnd(PIN_BATTERY);
		batteryVoltageInBits = basicFilter(analogRead(PIN_BATTERY), 0.95, batteryVoltageInBits);   // take smoothed adc value
		batteryVoltageInVolts = double(batteryVoltageInBits) * ((BAT_VOLT_DIV_R1 + BAT_VOLT_DIV_R2) / BAT_VOLT_DIV_R2) * 3.3 / 4095 * ADC_ERROR_FACTOR;
		//adcStart(PIN_BATTERY);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(50);
	}
	vTaskDelete(NULL);
}

void task_melody(void * parameter)
{
	while (true)
	{
		if (batteryVoltageInVolts >= BATT_LEVEL_EXIST && batteryVoltageInVolts <= BATT_LEVEL_CRITICAL)
			buzzer.play(buzzerMelodyBatLow);
		else if (modeQuad == modeQuadARMED && cmdMotorThr <= CMD_THR_ARM_START)
			buzzer.play(buzzerMelodyArmed);
		else if (modeQuad == modeQuadDirCmd && cmdMotorThr <= CMD_THR_ARM_START)
			buzzer.play(buzzerMelodyDirCmd);
		else
			buzzer.play(buzzerMelodySafe);
		delay(100);
	}
	vTaskDelete(NULL);
}


void task_altitude_kalman(void * parameter)
{
	while (millis() < KALMAN_TASK_START_TIME)
	{
		delay(200);
	}

	while (!baroReady)
	{
		delay(500);
	}

	double barometerAltReference = barometerAlt; // first baro altitude value is stored as a reference
	double barometerAltLocal = 0.0; // barometerAlt - barometerAltReference;

	double referenceAltitude = 0.0; // This variable is switched between ultrasonic sensor and barometer for Kalman implementation

	double T = 0.010; // Sampling Period

	//***********************************************************************************
	// Kalman Parameters for Single Altitude Estimation
	double deltaRowAlt = 1.0; // max(diff(baroAlt));
	double sigmaRowAlt = 1.0; // histogram(diff(baroAlt));

	double sigmaQ_Alt = deltaRowAlt;
	double Q_Alt = pow(sigmaQ_Alt, 2) * T; // Process noise covariance matrix

	double R_Alt = pow(sigmaRowAlt,2); // Measurement noise covariance matrix

	// Initialization
	double m_Alt_n1 = -referenceAltitude; // negative added since altitude vector is opposite of z-axis
	double P_Alt_n1 = pow(sigmaRowAlt, 2);

	double m_Alt_n = 0.0;
	double P_Alt_n = 0.0;
	//***********************************************************************************


	//***********************************************************************************
	// Kalman Parameters for Altitude, Velocity and Acceleration Estimation
	double F_AltVelAcc[9] = { 1, 0, 0, T, 1, 0, 1 / 2 * pow(T,2), T, 1 }; // State-transition matrix
	double H_AltVelAcc[6] = { 1, 0, 0, 0, 0, 1 }; // Measurement matrix

	double deltaAccelZ = 0.16;
	double sigmaQ_AltVelAcc = deltaAccelZ; // max(diff(accelerationZ));
	double Q_AltVelAcc[9] =  { 1/4*pow(T,4), 1 / 2 * pow(T,3), 1 / 2 * pow(T,2), 1 / 2 * pow(T,3), pow(T,2), T, 1 / 2 * pow(T,2), T, 1 };
	for (int i = 0; i < 9; i++) Q_AltVelAcc[i] *= pow(sigmaQ_AltVelAcc,2); // Process noise covariance matrix

	double sigmaAlt = 0.1; // histogram(diff(baroAlt));
	double sigmaAccelZ = 0.2; // histogram(diff(accelerationZ));
	double R_AltVelAcc[4] = { pow(sigmaAlt,2), 0, 0, pow(sigmaAccelZ,2) }; // Measurement noise covariance matrix

	// Initialization
	double m_AltVelAcc_n1[3] = { -referenceAltitude, 0, qc.accelWorld.z};  // negative added since altitude vector is opposite of z-axis
	double P_AltVelAcc_n1[9] = { pow(sigmaAlt,2), 0, 0, 0, pow(sigmaAccelZ,2), 0, 0, 0, pow(sigmaAccelZ,2) };

	double m_AltVelAcc_n[3] = { 0, 0, 0 };
	double P_AltVelAcc_n[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double y_AltVelAcc_n[2] = { 0, 0 };
	//***********************************************************************************

	// Initialize Kalman Filtering
	kalmanFilter_initialize();

	//unsigned long strtTime;
	
	while (true)
	{
		//strtTime = micros();

		//***********************************************************************************
		// Kalman Filter for Single Altitude Estimation
		barometerAltLocal = barometerAlt - barometerAltReference;
		kalmanFilterOneParameter(m_Alt_n1, P_Alt_n1, barometerAltLocal, 1.0, Q_Alt, 1.0, R_Alt, &m_Alt_n, &P_Alt_n);

		m_Alt_n1 = m_Alt_n;
		P_Alt_n1 = P_Alt_n;
		//***********************************************************************************

		if (ultrasonicDistanceFiltered > 0.01 && ultrasonicDistanceFiltered < 3.8) {  //ultrasonic sensor output equals to 0 if it is not connected
			referenceAltitude = ultrasonicDistanceFiltered;
		}
		else
		{
			referenceAltitude = m_Alt_n; // If cascaded Kalman is not used: barometerAltLocal;
		}


		//***********************************************************************************
		// Kalman Filter for Altitude, Velocity and Acceleration Estimation
		y_AltVelAcc_n[0] = -referenceAltitude; // negative added since altitude vector is opposite of z-axis
		y_AltVelAcc_n[1] = qc.accelWorld.z;

		kalmanFilter3State2Measurement(m_AltVelAcc_n1, P_AltVelAcc_n1, y_AltVelAcc_n, F_AltVelAcc, Q_AltVelAcc, H_AltVelAcc, R_AltVelAcc, m_AltVelAcc_n, P_AltVelAcc_n);
		
		qc.posWorldEstimated.z = m_AltVelAcc_n[0];     
		qc.velWorldEstimated.z = m_AltVelAcc_n[1];
		qc.accelWorldEstimated.z = m_AltVelAcc_n[2];

		memcpy(m_AltVelAcc_n1, m_AltVelAcc_n, sizeof(m_AltVelAcc_n));
		memcpy(P_AltVelAcc_n1, P_AltVelAcc_n, sizeof(P_AltVelAcc_n));
		//***********************************************************************************

		calculateAccelWorldZDifferentials();

		//Serial.println(micros() - strtTime);

		//Serial.println(millis() - lastTime);
		//lastTime = millis();

		delay(9);
	}
	vTaskDelete(NULL);
}

void task_position_kalman(void * parameter)
{

	while (millis() < KALMAN_TASK_START_TIME)
	{
		delay(200);
	}


	double T = 0.010; // Sampling Period


	//***********************************************************************************
	// Kalman Parameters for Position, Velocity and Acceleration Estimation
	double F_PosVelAcc[9] = { 1, 0, 0, T, 1, 0, 1 / 2 * pow(T,2), T, 1 }; // State-transition matrix
	
	double H_PosVelAcc[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 }; // Measurement matrix when gpsPos, gpsVel and Acc available
	double H_PosVelAcc_VelAcc[6] = { 0, 0, 1, 0, 0, 1 }; // Measurement matrix when only gpsVel and Acc available
	double H_PosVelAcc_Acc[3] = { 0, 0, 1 }; // Measurement matrix when only Acc available

	double deltaAccelXY = 0.05;
	double sigmaQ_PosVelAcc = deltaAccelXY;
	double Q_PosVelAcc[9] = { 1 / 4 * pow(T,4), 1 / 2 * pow(T,3), 1 / 2 * pow(T,2), 1 / 2 * pow(T,3), pow(T,2), T, 1 / 2 * pow(T,2), T, 1 };
	for (int i = 0; i < 9; i++) Q_PosVelAcc[i] *= pow(sigmaQ_PosVelAcc,2); // Process noise covariance matrix

	// Default measurement noise covariance matrix, will be updated in the loop
	double sigmaPos = 3;
	double sigmaVel = 0.3;
	double sigmaAccelXY = 0.05;
	double R_PosVelAcc[9] = { sigmaPos, 0, 0, 0, sigmaVel, 0, 0, 0, sigmaAccelXY }; // Measurement noise covariance matrix when gpsPos, gpsVel and Acc available
	double R_PosVelAcc_VelAcc[4] = { sigmaVel, 0, 0, sigmaAccelXY }; // Measurement noise covariance matrix when only gpsVel and Acc available
	double R_PosVelAcc_Acc = sigmaAccelXY; // Measurement noise covariance matrix when only Acc available

	// Initialization
	double m_PosVelAccN_n1[3] = { 0, 0, 0 };
	double m_PosVelAccE_n1[3] = { 0, 0, 0 };
	double P_PosVelAccN_n1[9] = { sigmaPos, 0, 0, 0, sigmaVel, 0, 0, 0, sigmaAccelXY };
	double P_PosVelAccE_n1[9] = { sigmaPos, 0, 0, 0, sigmaVel, 0, 0, 0, sigmaAccelXY };

	double m_PosVelAccN_n[3] = { 0, 0, 0 };
	double m_PosVelAccE_n[3] = { 0, 0, 0 };
	double P_PosVelAccN_n[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double P_PosVelAccE_n[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	
	double y_PosVelAccN_n[3] = { 0, 0, 0 };	
	double y_PosVelAccE_n[3] = { 0, 0, 0 };
	double y_PosVelAccN_VelAcc_n[2] = { 0, 0 };
	double y_PosVelAccE_VelAcc_n[2] = { 0, 0 };
	double y_PosVelAccN_Acc_n = 0;
	double y_PosVelAccE_Acc_n = 0;
	//***********************************************************************************

	// Initialize Kalman Filtering
	kalmanFilter_initialize();

	//unsigned long strtTime;

	while (true)
	{
		//strtTime = micros();

		//***********************************************************************************
		// Kalman Filter for Position, Velocity and Acceleration Estimation
		
		if (!gpsPositionAvailable && !gpsVelocityAvailable) {

			F_PosVelAcc[4] = 0.999; // Leaky Integral

			y_PosVelAccN_Acc_n = qc.accelNED.N;
			y_PosVelAccE_Acc_n = qc.accelNED.E;

			kalmanFilter3State1Measurement(m_PosVelAccN_n1, P_PosVelAccN_n1, y_PosVelAccN_Acc_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc_Acc, R_PosVelAcc_Acc, m_PosVelAccN_n, P_PosVelAccN_n);
			kalmanFilter3State1Measurement(m_PosVelAccE_n1, P_PosVelAccE_n1, y_PosVelAccE_Acc_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc_Acc, R_PosVelAcc_Acc, m_PosVelAccE_n, P_PosVelAccE_n);
		
		}
		else {

			F_PosVelAcc[4] = 1;

			if (gpsPositionAvailable && gpsVelocityAvailable) {

				y_PosVelAccN_n[0] = qcGPS.nedCoordinate.N;
				y_PosVelAccE_n[0] = qcGPS.nedCoordinate.E;

				y_PosVelAccN_n[1] = qcGPS.nedVelocity.N;
				y_PosVelAccE_n[1] = qcGPS.nedVelocity.E;

				y_PosVelAccN_n[2] = qc.accelNED.N;
				y_PosVelAccE_n[2] = qc.accelNED.E;

				sigmaPos = qcGPS.posAccuracy;
				sigmaVel = qcGPS.velAccuracy;
				R_PosVelAcc[0] = sigmaPos;
				R_PosVelAcc[4] = sigmaVel;

				kalmanFilter3State3Measurement(m_PosVelAccN_n1, P_PosVelAccN_n1, y_PosVelAccN_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc, R_PosVelAcc, m_PosVelAccN_n, P_PosVelAccN_n);
				kalmanFilter3State3Measurement(m_PosVelAccE_n1, P_PosVelAccE_n1, y_PosVelAccE_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc, R_PosVelAcc, m_PosVelAccE_n, P_PosVelAccE_n);

			}
			else if (!gpsPositionAvailable && gpsVelocityAvailable) {

				y_PosVelAccN_VelAcc_n[0] = qcGPS.nedVelocity.N;
				y_PosVelAccE_VelAcc_n[0] = qcGPS.nedVelocity.E;

				y_PosVelAccN_VelAcc_n[1] = qc.accelNED.N;
				y_PosVelAccE_VelAcc_n[1] = qc.accelNED.E;

				sigmaVel = qcGPS.velAccuracy;

				R_PosVelAcc_VelAcc[0] = sigmaVel;

				kalmanFilter3State2Measurement(m_PosVelAccN_n1, P_PosVelAccN_n1, y_PosVelAccN_VelAcc_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc_VelAcc, R_PosVelAcc_VelAcc, m_PosVelAccN_n, P_PosVelAccN_n);
				kalmanFilter3State2Measurement(m_PosVelAccE_n1, P_PosVelAccE_n1, y_PosVelAccE_VelAcc_n, F_PosVelAcc, Q_PosVelAcc, H_PosVelAcc_VelAcc, R_PosVelAcc_VelAcc, m_PosVelAccE_n, P_PosVelAccE_n);

			}

		}

		qc.posNEDEstimated.N = m_PosVelAccN_n[0];
		qc.velNEDEstimated.N = m_PosVelAccN_n[1];
		qc.accelNEDEstimated.N = m_PosVelAccN_n[2];

		qc.posNEDEstimated.E = m_PosVelAccE_n[0];
		qc.velNEDEstimated.E = m_PosVelAccE_n[1];
		qc.accelNEDEstimated.E = m_PosVelAccE_n[2];

		convertNed2World(qc.posNEDEstimated.N, qc.posNEDEstimated.E, compassHdgEstimated, &qc.posWorldEstimated.x, &qc.posWorldEstimated.y);
		convertNed2World(qc.velNEDEstimated.N, qc.velNEDEstimated.E, compassHdgEstimated, &qc.velWorldEstimated.x, &qc.velWorldEstimated.y);
		convertNed2World(qc.accelNEDEstimated.N, qc.accelNEDEstimated.E, compassHdgEstimated, &qc.accelWorldEstimated.x, &qc.accelWorldEstimated.y);

		memcpy(m_PosVelAccN_n1, m_PosVelAccN_n, sizeof(m_PosVelAccN_n));
		memcpy(P_PosVelAccN_n1, P_PosVelAccN_n, sizeof(P_PosVelAccN_n));
		memcpy(m_PosVelAccE_n1, m_PosVelAccE_n, sizeof(m_PosVelAccE_n));
		memcpy(P_PosVelAccE_n1, P_PosVelAccE_n, sizeof(P_PosVelAccE_n));

		//***********************************************************************************

		calculateAccelWorldXYDifferentials();

		//Serial.println(micros() - strtTime);

		delay(9);
	}
	vTaskDelete(NULL);
}

void task_compass_kalman(void * parameter)
{

	while (statusCompass != statusType_Normal)
	{
		delay(100);
	}


	double T = 0.010; // Sampling Period


	//***********************************************************************************
	// Kalman Parameters for Compass Heading Estimation
	double H_Heading[3] = { 1, 0, 0 }; // Measurement matrix

	double psdYaw = pow(0.015, 2); // Power Spectral Density (Variance) of Yaw Measurement
	double psdScale = pow(0.015, 2); // Power Spectral Density (Variance) of Scale Error
	double psdBias = pow(0.015, 2); // Power Spectral Density (Variance) of Bias Error

	double sigmaCompass = 0.75; // Standard Deviation of Compass Measurement;
	double R_Heading = pow(sigmaCompass, 2); // Measurement noise covariance matrix

											 // Initialization
	double m_Heading_n1[3] = { 0, 0, 0 };
	double P_Heading_n1[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

	double m_Heading_n[3] = { 0, 0, 0 };
	double P_Heading_n[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double y_Heading_n;

	double yawAngleOld = qc.euler.psi * 180 / M_PI;
	double yawAngleCurrent;
	double deltaYaw;
	//***********************************************************************************


	// Initialize Kalman Filtering
	kalmanFilter_initialize();


	while (true)
	{

		//***********************************************************************************
		// Kalman Filter for Compass Heading Estimation
		yawAngleCurrent = qc.euler.psi * 180 / M_PI;
		deltaYaw = yawAngleCurrent - yawAngleOld;
		if (deltaYaw > 180) deltaYaw -= 360;
		else if (deltaYaw <= -180) deltaYaw += 360;

		y_Heading_n = yawAngleCurrent - compassHdg;
		if (y_Heading_n > 180) y_Heading_n -= 360;
		else if (y_Heading_n <= -180) y_Heading_n += 360;

		// State-transition matrix
		double F_Heading[9] = { 1, 0, 0, deltaYaw, 1, 0, T, 0, 1 };

		// Process noise covariance matrix
		double Q_Heading[9] = { psdBias*pow(T, 3) / 3 + psdScale*pow(deltaYaw, 2)*T / 3 + psdYaw*T, deltaYaw*psdScale*T / 2, psdBias*pow(T, 2) / 2, deltaYaw*psdScale*T / 2, psdScale*T, 0, psdBias*pow(T, 2) / 2, 0, psdBias*T };

		kalmanFilter3State1Measurement(m_Heading_n1, P_Heading_n1, y_Heading_n, F_Heading, Q_Heading, H_Heading, R_Heading, m_Heading_n, P_Heading_n);

		compassHdgEstimated = yawAngleCurrent - m_Heading_n[0];
		if (compassHdgEstimated > 180) compassHdgEstimated -= 360;
		else if (compassHdgEstimated <= -180) compassHdgEstimated += 360;

		memcpy(m_Heading_n1, m_Heading_n, sizeof(m_Heading_n));
		memcpy(P_Heading_n1, P_Heading_n, sizeof(P_Heading_n));

		yawAngleOld = yawAngleCurrent;
		//***********************************************************************************


		delay(9);
	}
	vTaskDelete(NULL);
}

void task_IoT(void * parameter)
{
	WiFiClient clientIoT;

	// ThingSpeak Settings
	const int channelID = 287860;
	String writeAPIKey = "AF8MQWKYWKF7Z393"; // write API key for your ThingSpeak Channel
	const char* serverIoT = "api.thingspeak.com";

	while (true)
	{
		if (clientIoT.connect(serverIoT, 80)) {

			// Measure Signal Strength (RSSI) of Wi-Fi connection
			long rssi = WiFi.RSSI();

			// Construct API request body
			String body = "field1=";
			body += String(rssi);

			clientIoT.print("POST /update HTTP/1.1\n");
			clientIoT.print("Host: api.thingspeak.com\n");
			clientIoT.print("Connection: close\n");
			clientIoT.print("X-THINGSPEAKAPIKEY: " + writeAPIKey + "\n");
			clientIoT.print("Content-Type: application/x-www-form-urlencoded\n");
			clientIoT.print("Content-Length: ");
			clientIoT.print(body.length());
			clientIoT.print("\n\n");
			clientIoT.print(body);
			clientIoT.print("\n\n");
		}
		clientIoT.stop();
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(10*1000);
	}
	vTaskDelete(NULL);
}

void task_lidar(void * parameter)
{
	while (statusMpu != statusType_Normal)
	{
		delay(700);
	}


	lidar.begin(0, true);
	delay(10);
	lidar_distance = lidar.distance();


	if (lidar_distance > LIDAR_DISTANCE_MIN && lidar_distance < LIDAR_DISTANCE_MAX)
		lidar_available = true;
	else
		lidar_available = false;

	while (lidar_available)
	{

		lidar_distance=lidar.distance();


		if (lidar_distance > LIDAR_DISTANCE_MIN && lidar_distance < LIDAR_DISTANCE_MAX)
			lidar_available = true;
		else
			lidar_available = false;
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(50);
	}
	vTaskDelete(NULL);
	return;
}


void processTest() {

	//Serial.print("ADC Ch:");
	//Serial.print(digitalPinToAnalogChannel(PIN_BATTERY));
	//Serial.print("  Bat:");
	//Serial.println(batteryVoltageInBits);
	//Serial.print("DC_Roll:");
	//Serial.println(dutyCycle_Roll);
	digitalWrite(PIN_LED, HIGH);
	delay(750);
	digitalWrite(PIN_LED, LOW);
	delay(750);


	//if (WiFi.status() != WL_CONNECTED)
	//{
	//	Serial.println("wifi connection lost");
	//}
	
/*
	Serial.print("Temp");
	Serial.println(barometerTemp);*/

}

bool initMPU()
{
	digitalWrite(PIN_MPU_POWER_ON, LOW);
	delay(200);

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
		Serial.println("MPU6050 connection failed");
		return false;
	}
	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	mpu.setXAccelOffset(mpuXAccelOffset);
	mpu.setYAccelOffset(mpuYAccelOffset);
	mpu.setZAccelOffset(mpuZAccelOffset);
	mpu.setXGyroOffset(mpuXGyroOffset);
	mpu.setYGyroOffset(mpuYGyroOffset);
	mpu.setZGyroOffset(mpuZGyroOffset);

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

		// reset so we can continue cleanly
		mpu.resetFIFO();

		statusMpu = statusType_Normal;
		
	}
	else
	{
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));

		statusMpu = statusType_Fail;
	}
}

void processMpu()
{

	//get INT_STATUS byte
	mpuIntStatus = mpu.getIntStatus();

	//If mpu connection is lost, re initialize mpu, check the code mpuIntStatus statement again!
	if (mpuIntStatus == 0)
	{
		//initMPU();
		//Serial.println("bos veri");

	}
	// get current FIFO count
	fifoCount = mpu.getFIFOCount();
	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024)
	{
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));
		// otherwise, check for DMP data ready interrupt (this should happen frequently)

		//Serial.println("fifo overflow");
	}
	else if ((mpuIntStatus & 0x02))
	{
		//Serial.print("F:");
		//Serial.println(fifoCount);
		if (fifoCount >= packetSize)
		{

			//Serial.print(fifoCount);
			//Serial.print("  ");
			//Serial.println(millis());
			// wait for correct available data length, should be a VERY short wait
			//while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

			// read a packet from FIFO
			while (fifoCount >= packetSize)
			{
				mpu.getFIFOBytes(fifoBuffer, packetSize);
				// track FIFO count here in case there is > 1 packet available
				// (this lets us immediately read more without waiting for an interrupt)
				fifoCount -= packetSize;
			}

			mpuLastDataTime = millis();

			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGyro(gg, fifoBuffer);
			mpu.dmpGetEuler(euler, &q);

			qc.gyro.x = gg[0];
			qc.gyro.y = -gg[1];
			qc.gyro.z = -gg[2];

			calculateGyroDifferentials();

			qc.accel.x = aa.x;
			qc.accel.y = -aa.y;  // changed to negative to be consistent with standard a/c coordinate axis convention
			qc.accel.z = -aa.z;  // changed to negative to be consistent with standard a/c coordinate axis convention

			qc.euler.psi = -euler[0];
			qc.euler.theta = -euler[1];
			qc.euler.phi = euler[2];

			qc.accelWorld.x = (qc.accel.x*cos(qc.euler.theta) + qc.accel.z*cos(qc.euler.phi)*sin(qc.euler.theta) + qc.accel.y*sin(qc.euler.theta)*sin(qc.euler.phi)) * ACC_BITS_TO_M_SECOND2;
			qc.accelWorld.y = (qc.accel.y*cos(qc.euler.phi) - qc.accel.z*sin(qc.euler.phi)) * ACC_BITS_TO_M_SECOND2;
			qc.accelWorld.z = (qc.accel.z*cos(qc.euler.theta)*cos(qc.euler.phi) - qc.accel.x*sin(qc.euler.theta) + qc.accel.y*cos(qc.euler.theta)*sin(qc.euler.phi) + mpu_gravity_measurement_in_bits) * ACC_BITS_TO_M_SECOND2;

			convertWorld2Ned(qc.accelWorld.x, qc.accelWorld.y, compassHdgEstimated, &qc.accelNED.N, &qc.accelNED.E);
			//Serial.println(qc.gyro.x);
		}

	}
}

void processMapRxDCtoCmd()
{
	//Serial.println(cmdRxPitchCalibrated);
	if (statusRx == statusType_Normal)
	{
		cmdRxPitch = mapping(filterRxPitch.process(dutyCycle_Pitch), DC_PITCH_MIN, DC_PITCH_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);
		cmdRxRoll = mapping(filterRxRoll.process(dutyCycle_Roll), DC_ROLL_MIN, DC_ROLL_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);
		cmdRxYaw = mapping(filterRxYaw.process(dutyCycle_Yaw), DC_YAW_MIN, DC_YAW_MAX, CMD_YAW_MIN, CMD_YAW_MAX);
		cmdRxThr = mapping(filterRxThr.process(dutyCycle_Thr), DC_THR_MIN, DC_THR_MAX, CMD_THR_MIN, CMD_THR_MAX);

		cmdRx5thCh = mapping(filterRx5thCh.process(dutyCycle_Rx5thCh), DC_5TH_CH_MIN, DC_5TH_CH_MAX, -CMD_5TH_CH_MAX, CMD_5TH_CH_MAX);
		cmdRx6thCh = mapping(filterRx6thCh.process(dutyCycle_Rx6thCh), DC_6TH_CH_MIN, DC_6TH_CH_MAX, -CMD_6TH_CH_MAX, CMD_6TH_CH_MAX);



		//This below code segment is written to have smoother and much feasible commands for quadcopter
#ifdef COMMAND_CALIBRATION
		if (abs(cmdRxRoll) > 2)
		{
			cmdRxPitchRollAngle = atan(abs(cmdRxPitch / cmdRxRoll));
			cmdRxPitchRollSF = cos(_min(cmdRxPitchRollAngle, M_PI_2 - cmdRxPitchRollAngle));


			cmdRxPitchRollXfactor = sin(CMD_MAX_ATTITUDE_IN_RADIANS);




			cmdRxRollCalibratedInRad = cmdRxRoll / CMD_RX_PITCH_ROLL_MAX * cmdRxPitchRollSF * cmdRxPitchRollXfactor;
			cmdRxPitchCalibratedInRad = cmdRxPitch / CMD_RX_PITCH_ROLL_MAX * cmdRxPitchRollSF * cmdRxPitchRollXfactor;

			cmdRxRollCalibratedInRad = asin(cmdRxRollCalibratedInRad);
			cmdRxPitchCalibratedInRad = asin(cmdRxPitchCalibratedInRad / cos(cmdRxRollCalibratedInRad));


			if (cmdRxPitch >= 0)
				cmdRxPitchCalibrated = abs(cmdRxPitchCalibratedInRad) * 180.0 / M_PI;
			else
				cmdRxPitchCalibrated = -abs(cmdRxPitchCalibratedInRad) * 180.0 / M_PI;

			if (cmdRxRoll >= 0)
				cmdRxRollCalibrated = abs(cmdRxRollCalibratedInRad) * 180.0 / M_PI;
			else
				cmdRxRollCalibrated = -abs(cmdRxRollCalibratedInRad) * 180.0 / M_PI;

		}
		else
		{
			cmdRxPitchCalibrated = cmdRxPitch;
			cmdRxRollCalibrated = cmdRxRoll;

		}
#endif  //COMMAND_CALIBRATION



	}
	else
	{
		cmdRxPitch = 0;
		cmdRxRoll = 0;
		cmdRxYaw = 0;
		cmdRxThr = CMD_THR_MIN;
		cmdRx5thCh = 0;
		cmdRx6thCh = 0;

		cmdRxPitchCalibrated = 0;
		cmdRxRollCalibrated = 0;
	}

}

float mapping(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
	float result = 0;

	if (input <= inputMin)
	{
		result = outputMin;
	}
	else if (input >= inputMax)
	{
		result = outputMax;
	}
	else
	{
		result = outputMin + (input - inputMin) / (inputMax - inputMin) * (outputMax - outputMin);
	}
	return result;
}

void processCheckMode()
{
	if (statusRx != statusType_Normal)
	{
		modeQuad = modeQuadSAFE;
		//Serial.println("QUAD is in SAFE Mode Check Rx!");
		return;
	}

	if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw<CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll > CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadPreARM;
		//Serial.println("QUAD is ARMED");
	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll < -CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadSAFE;
		//Serial.println("QUAD is in SAFE Mode");

	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll < -CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch<-CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP)
	{
		//Left for spare usage
	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch<-CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadDirCmd;
	}

	if (modeQuad == modeQuadPreARM)
	{
		if (gravityAcquired && (!qcGPS.gpsIsFix || homePointSelected))
		{
			modeQuad = modeQuadARMED;
		}
		else
		{
			//if required conditions met (drone is standing still), get gravity measurement
			if (abs(qc.gyro.x) < 3 && abs(qc.gyro.y) < 3 && abs(qc.gyro.z) < 3 && !gravityAcquired)
			{
				mpu_gravity_measurement_in_bits = sqrt(pow(qc.accel.x, 2) + pow(qc.accel.y, 2) + pow(qc.accel.z, 2));

				gravityAcquired = true;
			}

			if (qcGPS.gpsIsFix && !homePointSelected)
			{

				homePoint.ecefCoordinate.x = qcGPS.ecefCoordinate.x;
				homePoint.ecefCoordinate.y = qcGPS.ecefCoordinate.y;
				homePoint.ecefCoordinate.z = qcGPS.ecefCoordinate.z;
				homePoint.lat = qcGPS.lat;
				homePoint.lon = qcGPS.lon;
				homePoint.alt = qcGPS.alt;

				homePointSelected = true;
			}

			
		}

	}



	if (modeQuad == modeQuadARMED && cmdMotorThr > CMD_THR_TAKEOFF)
	{
		pidRatePitch.SetFlightMode(true);
		pidRateRoll.SetFlightMode(true);
		pidRateYaw.SetFlightMode(true);
		pidAnglePitch.SetFlightMode(true);
		pidAngleRoll.SetFlightMode(true);
		pidAngleYaw.SetFlightMode(true);
		pidPosAlt.SetFlightMode(true);
		pidVelAlt.SetFlightMode(true);
		pidAccAlt.SetFlightMode(true);   //this will be discussed later
		pidAccX.SetFlightMode(true);
		pidAccY.SetFlightMode(true);
		pidVelX.SetFlightMode(true);
		pidVelY.SetFlightMode(true);
		pidPosX.SetFlightMode(true);
		pidPosY.SetFlightMode(true);
	}
	else
	{
		pidRatePitch.SetFlightMode(false);
		pidRateRoll.SetFlightMode(false);
		pidRateYaw.SetFlightMode(false);
		pidAnglePitch.SetFlightMode(false);
		pidAngleRoll.SetFlightMode(false);
		pidAngleYaw.SetFlightMode(false);
		pidPosAlt.SetFlightMode(false);
		pidVelAlt.SetFlightMode(false);
		//pidAccAlt.SetFlightMode(false);   //this will be discussed later
		pidAccX.SetFlightMode(false);
		pidAccY.SetFlightMode(false);
		pidVelX.SetFlightMode(false);
		pidVelY.SetFlightMode(false);
		pidPosX.SetFlightMode(false);
		pidPosY.SetFlightMode(false);
	}
}

void processPID()
{
	prePIDprocesses();

	// getBodyToEulerAngularRates(); // This function is not necessary for this task

	calculatePosCmdXYDifferentials();

	// Pos X PID
	pidVars.posX.setpoint = posCmd.x; // meters
	pidVars.posX.sensedVal = qc.posWorldEstimated.x; // meters
	pidVars.posX.setpointDiff = posCmdDiff.x;
	pidVars.posX.sensedValDiff = qc.velWorldEstimated.x; // m/s
	pidPosX.Compute();

	// Pos Y PID
	pidVars.posY.setpoint = posCmd.y; // meters
	pidVars.posY.sensedVal = qc.posWorldEstimated.y; // meters
	pidVars.posY.setpointDiff = posCmdDiff.y;
	pidVars.posY.sensedValDiff = qc.velWorldEstimated.y; // m/s
	pidPosY.Compute();

	// filterPosXYPIDoutputs(); // This filtering is not necessary

	if ((cmdMotorPitch == 0) && posHoldAvailable) // Calculate Velocity X Command (When Auto Mode)
	{
		velCmd.x = pidVars.posX.output; // Filtered output is not necessary
	}
	else
	{
		velCmd.x = -cmdMotorPitch * 10;   // negative added since rx pitch command is in the reverse direction of x-axis
		posCmd.x = qc.posWorldEstimated.x;
	}

	if ((cmdMotorRoll == 0) && posHoldAvailable) // Calculate Velocity Y Command (When Auto Mode)
	{
		velCmd.y = pidVars.posY.output; // Filtered output is not necessary
	}
	else
	{
		velCmd.y = cmdMotorRoll * 10;
		posCmd.y = qc.posWorldEstimated.y;
	}

	calculateVelCmdXYDifferentials();

	// Vel X PID
	pidVars.velX.setpoint = velCmd.x;
	pidVars.velX.sensedVal = qc.velWorldEstimated.x * 100;  // cm/s
	pidVars.velX.setpointDiff = velCmdDiff.x;
	pidVars.velX.sensedValDiff = qc.accelWorldEstimated.x * 100;   // cm/s^2 
	pidVelX.Compute();

	// Vel Y PID
	pidVars.velY.setpoint = velCmd.y;
	pidVars.velY.sensedVal = qc.velWorldEstimated.y * 100;  // cm/s
	pidVars.velY.setpointDiff = velCmdDiff.y;
	pidVars.velY.sensedValDiff = qc.accelWorldEstimated.y * 100;   // cm/s^2 
	pidVelY.Compute();

	// filterVelXYPIDoutputs(); // This filtering is not necessary

	// Calculate Acceleration X and Y Commands (When Auto Mode)
	accelCmd.x = pidVars.velX.output; // Filtered output is not necessary
	accelCmd.y = pidVars.velY.output; // Filtered output is not necessary
	
	// Calculate Acceleration X and Y Commands (If Acc Control Mode)
	// accelCmd.x = -cmdMotorPitch * 20;  // negative added since rx pitch command is in the reverse direction of x-axis
	// accelCmd.y = cmdMotorRoll * 20;
	
	calculateAccelCmdXYDifferentials();

		// Acc X PID
	pidVars.accX.setpoint = accelCmd.x;
	pidVars.accX.sensedVal = qc.accelWorldEstimated.x * 100;  // cm/s^2 will be changed to transformed variable
	pidVars.accX.setpointDiff = accelCmdDiff.x;
	pidVars.accX.sensedValDiff = qc.accelWorldDiff.x * 100;   // cm/s^3 
	pidAccX.Compute();

	// Acc Y PID
	pidVars.accY.setpoint = accelCmd.y;
	pidVars.accY.sensedVal = qc.accelWorldEstimated.y * 100;  // cm/s^2 will be changed to transformed variable
	pidVars.accY.setpointDiff = accelCmdDiff.y;
	pidVars.accY.sensedValDiff = qc.accelWorldDiff.y * 100;   // cm/s^3 
	pidAccY.Compute();

	// filterAccXYPIDoutputs(); // This filtering is not necessary

	////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Pitch Roll Yaw Angle PID
	
	if (velHoldAvailable)
	{
		// Calculate Angle Commands (When Auto Mode)
		if (qc.accelWorld.z == GRAVITY_IN_METER_PER_SECOND2) { // Free-fall detection
			angleCmd.x = 0;
			angleCmd.y = 0;
		}
		else {
			angleCmd.y = -atan(pidVars.accX.output / ((GRAVITY_IN_METER_PER_SECOND2 - qc.accelWorld.z) * 100)) * 180 / M_PI;
			if (angleCmd.y > CMD_AUTO_PITCH_ROLL_MAX) angleCmd.y = CMD_AUTO_PITCH_ROLL_MAX;
			else if (angleCmd.y < -CMD_AUTO_PITCH_ROLL_MAX) angleCmd.y = -CMD_AUTO_PITCH_ROLL_MAX;

			angleCmd.x = atan(pidVars.accY.output * cos(angleCmd.y * M_PI / 180) / ((GRAVITY_IN_METER_PER_SECOND2 - qc.accelWorld.z) * 100)) * 180 / M_PI;
			if (angleCmd.x > CMD_AUTO_PITCH_ROLL_MAX) angleCmd.x = CMD_AUTO_PITCH_ROLL_MAX;
			else if (angleCmd.x < -CMD_AUTO_PITCH_ROLL_MAX) angleCmd.x = -CMD_AUTO_PITCH_ROLL_MAX;	
		}
	}
	else
	{
		// Calculate Angle Commands (When Manual Mode)
		angleCmd.x = cmdMotorRoll;
		angleCmd.y = cmdMotorPitch;
	}

	calculateAngleCmdDifferentials();

	// Roll PID	
	pidVars.angleRoll.setpoint = angleCmd.x;
	pidVars.angleRoll.sensedVal = qc.euler.phi * 180 / M_PI;
	pidVars.angleRoll.setpointDiff = angleCmdDiff.x;
	pidVars.angleRoll.sensedValDiff = qc.gyro.x;
	pidAngleRoll.Compute();
	
	// Pitch PID
	pidVars.anglePitch.setpoint = angleCmd.y;
	pidVars.anglePitch.sensedVal = qc.euler.theta * 180 / M_PI;
	pidVars.anglePitch.setpointDiff = angleCmdDiff.y;
	pidVars.anglePitch.sensedValDiff = qc.gyro.y;
	pidAnglePitch.Compute();

	// Yaw PID
	pidVars.angleYaw.setpoint = cmdMotorYaw;
	pidVars.angleYaw.sensedVal = qc.euler.psi * 180 / M_PI;
	pidVars.angleYaw.sensedValDiff = qc.gyro.z;
	pidAngleYaw.Compute();

	// filterAnglePIDoutputs(); // This filtering is not necessary

	// Rate Commands
	rateCmd.x = pidVars.angleRoll.output;
	rateCmd.y = pidVars.anglePitch.output;
	rateCmd.z = pidVars.angleYaw.output;

	calculateRateCmdDifferentials();
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////
	//Pitch Roll Yaw Rate PID
	
	//Pitch Rate PID
	pidVars.ratePitch.setpoint = rateCmd.y;
	pidVars.ratePitch.sensedVal = qc.gyro.y;
	pidVars.ratePitch.setpointDiff = rateCmdDiff.y;
	pidVars.ratePitch.sensedValDiff = qc.gyroDiff.y;
	pidRatePitch.Compute();

	//Roll Rate PID
	pidVars.rateRoll.setpoint = rateCmd.x;
	pidVars.rateRoll.sensedVal = qc.gyro.x;
	pidVars.rateRoll.setpointDiff = rateCmdDiff.x;
	pidVars.rateRoll.sensedValDiff = qc.gyroDiff.x;
	pidRateRoll.Compute();

	//Yaw Rate PID
	pidVars.rateYaw.setpoint = rateCmd.z;
	pidVars.rateYaw.sensedVal = qc.gyro.z;
	pidVars.rateYaw.setpointDiff = rateCmdDiff.z;
	pidVars.rateYaw.sensedValDiff = qc.gyroDiff.z;
	pidRateYaw.Compute();
	////////////////////////////////////////////////////////////////////////////////////////////////////////


	//Position Altitude PID

	calculatePosCmdZDifferentials();

	pidVars.posAlt.setpoint = posCmd.z; //meters
	pidVars.posAlt.sensedVal = -qc.posWorldEstimated.z;  // meters                  // negative added since altitude vector is opposite of z-axis
	pidVars.posAlt.setpointDiff = posCmdDiff.z;
	pidVars.posAlt.sensedValDiff = -qc.velWorldEstimated.z;  // m/s                 // negative added since altitude vector is opposite of z-axis
	pidPosAlt.Compute();

	// filterPosAltPIDoutputs(); // This filtering is not necessary
	
	// Velocity Commands
	if (getAltVelCmd(cmdRxThr) == 0)
	{
		velCmd.z = pidVars.posAlt.output; // Filtered output is not necessary
	}
	else
	{
		velCmd.z = getAltVelCmd(cmdRxThr);
		posCmd.z = -qc.posWorldEstimated.z;   // negative added since altitude vector is opposite of z-axis
	}

	calculateVelCmdZDifferentials();

	// Velocity Altitude PID
	pidVars.velAlt.setpoint = velCmd.z;  // cm/second
	pidVars.velAlt.sensedVal = -qc.velWorldEstimated.z * 100;  // cm/second           // negative added since altitude vector is opposite of z-axis
	pidVars.velAlt.setpointDiff = velCmdDiff.z;
	pidVars.velAlt.sensedValDiff = -qc.accelWorldEstimated.z * 100;  // cm/second^2   // negative added since altitude vector is opposite of z-axis
	pidVelAlt.Compute();

	// filterVelAltPIDoutputs(); // This filtering is not necessary

	//Transform vel pid outputs to body coordinate axis in order to get correct acceleration wrt world
	//transformVelPIDoutputsToBody();

	// Acceleration Z Commands
	accelCmd.z = pidVars.velAlt.output; // Filtered output is not necessary
	
	calculateAccelCmdZDifferentials();

	// Acceleration Altitude PID
	pidVars.accAlt.setpoint = accelCmd.z;   // cm/second^2
	pidVars.accAlt.sensedVal = -qc.accelWorldEstimated.z * 100;  // cm/second^2   // negative added since altitude vector is opposite of z-axis
	pidVars.accAlt.setpointDiff = accelCmdDiff.z;          // cm/second^3
	pidVars.accAlt.sensedValDiff = -qc.accelWorldDiff.z * 100;  // cm/second^3         // negative added since altitude vector is opposite of z-axis
	pidAccAlt.Compute();

	postPIDprocesses();
}

void initPIDvariables()
{
	//PID related variable initializations
	pidVars.ratePitch.Kp = PID_RATE_PITCH_KP;
	pidVars.ratePitch.Ki = PID_RATE_PITCH_KI;
	pidVars.ratePitch.Kd = PID_RATE_PITCH_KD;
	pidVars.ratePitch.outputLimitMin = PID_RATE_PITCH_OUTMIN;
	pidVars.ratePitch.outputLimitMax = PID_RATE_PITCH_OUTMAX;
	pidVars.ratePitch.output = 0;
	pidVars.ratePitch.outputCompensated = 0;

	pidVars.anglePitch.Kp = PID_ANGLE_PITCH_KP;
	pidVars.anglePitch.Ki = PID_ANGLE_PITCH_KI;
	pidVars.anglePitch.Kd = PID_ANGLE_PITCH_KD;
	pidVars.anglePitch.outputLimitMin = PID_ANGLE_PITCH_OUTMIN;
	pidVars.anglePitch.outputLimitMax = PID_ANGLE_PITCH_OUTMAX;
	pidVars.anglePitch.output = 0;
	pidVars.anglePitch.outputCompensated = 0;

	pidVars.rateRoll.Kp = PID_RATE_ROLL_KP;
	pidVars.rateRoll.Ki = PID_RATE_ROLL_KI;
	pidVars.rateRoll.Kd = PID_RATE_ROLL_KD;
	pidVars.rateRoll.outputLimitMin = PID_RATE_ROLL_OUTMIN;
	pidVars.rateRoll.outputLimitMax = PID_RATE_ROLL_OUTMAX;
	pidVars.rateRoll.output = 0;
	pidVars.rateRoll.outputCompensated = 0;

	pidVars.angleRoll.Kp = PID_ANGLE_ROLL_KP;
	pidVars.angleRoll.Ki = PID_ANGLE_ROLL_KI;
	pidVars.angleRoll.Kd = PID_ANGLE_ROLL_KD;
	pidVars.angleRoll.outputLimitMin = PID_ANGLE_ROLL_OUTMIN;
	pidVars.angleRoll.outputLimitMax = PID_ANGLE_ROLL_OUTMAX;
	pidVars.angleRoll.output = 0;
	pidVars.angleRoll.outputCompensated = 0;

	pidVars.rateYaw.Kp = PID_RATE_YAW_KP;
	pidVars.rateYaw.Ki = PID_RATE_YAW_KI;
	pidVars.rateYaw.Kd = PID_RATE_YAW_KD;
	pidVars.rateYaw.outputLimitMin = PID_RATE_YAW_OUTMIN;
	pidVars.rateYaw.outputLimitMax = PID_RATE_YAW_OUTMAX;
	pidVars.rateYaw.output = 0;
	pidVars.rateYaw.outputCompensated = 0;

	pidVars.angleYaw.Kp = PID_ANGLE_YAW_KP;
	pidVars.angleYaw.Ki = PID_ANGLE_YAW_KI;
	pidVars.angleYaw.Kd = PID_ANGLE_YAW_KD;
	pidVars.angleYaw.outputLimitMin = PID_ANGLE_YAW_OUTMIN;
	pidVars.angleYaw.outputLimitMax = PID_ANGLE_YAW_OUTMAX;
	pidVars.angleYaw.output = 0;
	pidVars.angleYaw.outputCompensated = 0;

	pidVars.posAlt.Kp = PID_POS_ALT_KP;
	pidVars.posAlt.Ki = PID_POS_ALT_KI;
	pidVars.posAlt.Kd = PID_POS_ALT_KD;
	pidVars.posAlt.outputLimitMin = PID_POS_ALT_OUTMIN;
	pidVars.posAlt.outputLimitMax = PID_POS_ALT_OUTMAX;
	pidVars.posAlt.output = 0;
	pidVars.posAlt.outputCompensated = 0;
	
	pidVars.velAlt.Kp = PID_VEL_ALT_KP;
	pidVars.velAlt.Ki = PID_VEL_ALT_KI;
	pidVars.velAlt.Kd = PID_VEL_ALT_KD;
	pidVars.velAlt.outputLimitMin = PID_VEL_ALT_OUTMIN;
	pidVars.velAlt.outputLimitMax = PID_VEL_ALT_OUTMAX;
	pidVars.velAlt.output = 0;
	pidVars.velAlt.outputCompensated = 0;

	pidVars.accAlt.Kp = PID_ACC_ALT_KP;
	pidVars.accAlt.Ki = PID_ACC_ALT_KI;
	pidVars.accAlt.Kd = PID_ACC_ALT_KD;
	pidVars.accAlt.outputLimitMin = CMD_THR_MIN;
	pidVars.accAlt.outputLimitMax = CMD_THR_MAX;
	pidVars.accAlt.output = 0;
	pidVars.accAlt.outputCompensated = 0;
	
	pidVars.accX.Kp = PID_ACC_X_KP;
	pidVars.accX.Ki = PID_ACC_X_KI;
	pidVars.accX.Kd = PID_ACC_X_KD;
	pidVars.accX.outputLimitMin = PID_ACC_X_OUTMIN;
	pidVars.accX.outputLimitMax = PID_ACC_X_OUTMAX;
	pidVars.accX.output = 0;
	pidVars.accX.outputCompensated = 0;

	pidVars.accY.Kp = PID_ACC_Y_KP;
	pidVars.accY.Ki = PID_ACC_Y_KI;
	pidVars.accY.Kd = PID_ACC_Y_KD;
	pidVars.accY.outputLimitMin = PID_ACC_Y_OUTMIN;
	pidVars.accY.outputLimitMax = PID_ACC_Y_OUTMAX;
	pidVars.accY.output = 0;
	pidVars.accY.outputCompensated = 0;
	
	pidVars.velX.Kp = PID_VEL_X_KP;
	pidVars.velX.Ki = PID_VEL_X_KI;
	pidVars.velX.Kd = PID_VEL_X_KD;
	pidVars.velX.outputLimitMin = PID_VEL_X_OUTMIN;
	pidVars.velX.outputLimitMax = PID_VEL_X_OUTMAX;
	pidVars.velX.output = 0;
	pidVars.velX.outputCompensated = 0;

	pidVars.velY.Kp = PID_VEL_Y_KP;
	pidVars.velY.Ki = PID_VEL_Y_KI;
	pidVars.velY.Kd = PID_VEL_Y_KD;
	pidVars.velY.outputLimitMin = PID_VEL_Y_OUTMIN;
	pidVars.velY.outputLimitMax = PID_VEL_Y_OUTMAX;
	pidVars.velY.output = 0;
	pidVars.velY.outputCompensated = 0;

	pidVars.posX.Kp = PID_POS_X_KP;
	pidVars.posX.Ki = PID_POS_X_KI;
	pidVars.posX.Kd = PID_POS_X_KD;
	pidVars.posX.outputLimitMin = PID_POS_X_OUTMIN;
	pidVars.posX.outputLimitMax = PID_POS_X_OUTMAX;
	pidVars.posX.output = 0;
	pidVars.posX.outputCompensated = 0;

	pidVars.posY.Kp = PID_POS_Y_KP;
	pidVars.posY.Ki = PID_POS_Y_KI;
	pidVars.posY.Kd = PID_POS_Y_KD;
	pidVars.posY.outputLimitMin = PID_POS_Y_OUTMIN;
	pidVars.posY.outputLimitMax = PID_POS_Y_OUTMAX;
	pidVars.posY.output = 0;
	pidVars.posY.outputCompensated = 0;

}

void initPIDtuning()
{
	pidRatePitch.SetOutputLimits(pidVars.ratePitch.outputLimitMin, pidVars.ratePitch.outputLimitMax);
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);

	pidAnglePitch.SetOutputLimits(pidVars.anglePitch.outputLimitMin, pidVars.anglePitch.outputLimitMax);
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);

	pidRateRoll.SetOutputLimits(pidVars.rateRoll.outputLimitMin, pidVars.rateRoll.outputLimitMax);
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);

	pidAngleRoll.SetOutputLimits(pidVars.angleRoll.outputLimitMin, pidVars.angleRoll.outputLimitMax);
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);

	pidRateYaw.SetOutputLimits(pidVars.rateYaw.outputLimitMin, pidVars.rateYaw.outputLimitMax);
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);

	pidAngleYaw.SetOutputLimits(pidVars.angleYaw.outputLimitMin, pidVars.angleYaw.outputLimitMax);
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);

	pidPosAlt.SetOutputLimits(pidVars.posAlt.outputLimitMin, pidVars.posAlt.outputLimitMax);
	pidPosAlt.SetTunings(pidVars.posAlt.Kp, pidVars.posAlt.Ki, pidVars.posAlt.Kd);

	pidVelAlt.SetOutputLimits(pidVars.velAlt.outputLimitMin, pidVars.velAlt.outputLimitMax);
	pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);

	pidAccAlt.SetOutputLimits(pidVars.accAlt.outputLimitMin, pidVars.accAlt.outputLimitMax);
	pidAccAlt.SetTunings(pidVars.accAlt.Kp * DRONE_WEIGHT / 1000.0, pidVars.accAlt.Ki * DRONE_WEIGHT / 1000.0, pidVars.accAlt.Kd * DRONE_WEIGHT / 1000.0);

	pidAccX.SetOutputLimits(pidVars.accX.outputLimitMin, pidVars.accX.outputLimitMax);
	pidAccX.SetTunings(pidVars.accX.Kp, pidVars.accX.Ki, pidVars.accX.Kd);

	pidAccY.SetOutputLimits(pidVars.accY.outputLimitMin, pidVars.accY.outputLimitMax);
	pidAccY.SetTunings(pidVars.accY.Kp, pidVars.accY.Ki, pidVars.accY.Kd);

	pidVelX.SetOutputLimits(pidVars.velX.outputLimitMin, pidVars.velX.outputLimitMax);
	pidVelX.SetTunings(pidVars.velX.Kp, pidVars.velX.Ki, pidVars.velX.Kd);

	pidVelY.SetOutputLimits(pidVars.velY.outputLimitMin, pidVars.velY.outputLimitMax);
	pidVelY.SetTunings(pidVars.velY.Kp, pidVars.velY.Ki, pidVars.velY.Kd);

	pidPosX.SetOutputLimits(pidVars.posX.outputLimitMin, pidVars.posX.outputLimitMax);
	pidPosX.SetTunings(pidVars.posX.Kp, pidVars.posX.Ki, pidVars.posX.Kd);

	pidPosY.SetOutputLimits(pidVars.posY.outputLimitMin, pidVars.posY.outputLimitMax);
	pidPosY.SetTunings(pidVars.posY.Kp, pidVars.posY.Ki, pidVars.posY.Kd);
}

void prePIDprocesses()
{
	cmdMotorPitch = cmdRxPitchCalibrated;
	cmdMotorRoll = cmdRxRollCalibrated;



	if (modeQuad == modeQuadARMED && cmdMotorThr > CMD_THR_TAKEOFF)
	{
		if (abs(cmdRxYaw) > 6)
		{
			cmdMotorYaw += cmdRxYaw / 60.0;

			if (cmdMotorYaw > 180)
				cmdMotorYaw -= 360;
			else if (cmdMotorYaw <= -180)
				cmdMotorYaw += 360;
		}
	}
	else
	{
		cmdMotorYaw = qc.euler.psi * 180 / M_PI;
	}
}

void postPIDprocesses()
{
	if (autoModeStatus == autoModeAltitude)
	{
		cmdMotorThr = pidVars.accAlt.output;
	}
	else
	{
		posCmd.z = -qc.posWorldEstimated.z;   // negative added since altitude vector is opposite of z-axis
		cmdMotorThr = cmdRxThr;
		pidAccAlt.Set_I_Result(cmdMotorThr);
	}
}

void getBodyToEulerAngularRates()
{

	if (abs(cos(qc.euler.theta)) > 0.0001)
	{
		qc.eulerRate.phi = qc.gyro.x + sin(qc.euler.phi) * tan(qc.euler.theta) * qc.gyro.y + cos(qc.euler.phi) * tan(qc.euler.theta) * qc.gyro.z;
		qc.eulerRate.theta = cos(qc.euler.phi)*qc.gyro.y - sin(qc.euler.phi)*qc.gyro.z;
		qc.eulerRate.psi = sin(qc.euler.phi) / cos(qc.euler.theta) * qc.gyro.y + cos(qc.euler.phi) / cos(qc.euler.theta)*qc.gyro.z;
	}


}

double basicFilter(double data, double filterConstant, double filteredData)
{
	if (filterConstant > 1) {      // check to make sure param's are within range
		filterConstant = .99;
	}
	else if (filterConstant <= 0) {
		filterConstant = 0;
	}
	filteredData = (data * (1 - filterConstant)) + (filteredData  *  filterConstant);
	return filteredData;
}

void transformVelPIDoutputsToBody()
{

	//this code will be modified according to the behaviour of quadcopter during flight tests
	//double tempVar = cos(mpu.euler.phi)*cos(mpu.euler.theta);
	//if (abs(tempVar) > 0.5)
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / tempVar) + 500;
	//}
	//else if (tempVar >= 0)
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / 0.5) + 500;
	//}
	//else
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / -0.5) -500;   //this condition should be discussed
	//}


}

void handleAutoModeCommands()
{
#ifdef MY_RX_TX_IS_6_CHANNEL
	if (modeQuad == modeQuadARMED && (statusRx == statusType_Normal) && (cmdRx5thCh > (CMD_5TH_CH_MAX - 10)))
	{
		autoModeStatus = autoModeAltitude;
	}
	else
	{
		autoModeStatus = autoModeOFF;
	}

	if (modeQuad == modeQuadARMED && (statusRx == statusType_Normal) && (cmdRx6thCh > (CMD_6TH_CH_MAX - 10)))
	{
		velHoldAvailable = true;
		if (gpsPositionAvailable) {
			posHoldAvailable = true;
		}
		else {
			posHoldAvailable = false;
		}
	}
	else
	{
		velHoldAvailable = false;
		posHoldAvailable = false;
	}

#else	
	if (modeQuad == modeQuadARMED && MsgT01.message.coWorkerTxPacket.statusGS == statusType_Normal)
	{
		autoModeStatus = MsgUdpT01.message.autoModeCommand;
	}
	else
	{
		autoModeStatus = autoModeOFF;
	}

#endif // MY_RX_TX_IS_6_CHANNEL

}

void processRunMotors()
{

	if (modeQuad == modeQuadARMED)//modeQuad == modeQuadARMED
	{
		if (cmdMotorThr > CMD_THR_ARM_START)
		{
			calculate_pid_thr_batt_scale_factor();

			pidVars.ratePitch.outputCompensated = pidVars.ratePitch.output* PID_THR_BATT_SCALE_FACTOR;
			pidVars.rateRoll.outputCompensated = pidVars.rateRoll.output*PID_THR_BATT_SCALE_FACTOR;
			pidVars.rateYaw.outputCompensated = pidVars.rateYaw.output*PID_THR_BATT_SCALE_FACTOR;
		   
			
			//Serial.print("");
			//Serial.println(pidVars.rateYaw.outputCompensated);
			

			pwmMicroSeconds(M_FL_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_FR_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BR_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BL_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);



			//pwmMicroSeconds(M_FL_CHANNEL, cmdMotorThr);
			//pwmMicroSeconds(M_FR_CHANNEL, cmdMotorThr);
			//pwmMicroSeconds(M_BR_CHANNEL, cmdMotorThr);
			//pwmMicroSeconds(M_BL_CHANNEL, cmdMotorThr);

		}
		else
		{
			pwmMicroSeconds(M_FL_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_FR_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_BR_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_BL_CHANNEL, CMD_THR_MIN);
		}
	}
	else if (modeQuad == modeQuadDirCmd)
	{
		pwmMicroSeconds(M_FL_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_FR_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_BR_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_BL_CHANNEL, cmdRxThr);
	}
	else
	{
		pwmMicroSeconds(M_FL_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_FR_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_BR_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_BL_CHANNEL, CMD_THR_MIN);
	}

}

void calculate_pid_thr_batt_scale_factor()
{
	float total_thr_cmd = cmdMotorThr;

	if (total_thr_cmd <= CMD_THR_MAX && total_thr_cmd >= CMD_THR_MIN)
		PID_THR_BATT_SCALE_FACTOR = (CMD_THR_MAX - total_thr_cmd) / (CMD_THR_MAX - CMD_THR_MIN) + 0.2;
	else
		PID_THR_BATT_SCALE_FACTOR = 0.4;

	//Use linear interpolation for battery voltage level compensation
	//Make sure that voltage read is in valid range of operation
	if (batteryVoltageInVolts > 9.0 && batteryVoltageInVolts < 13.2)
	{		
		PID_THR_BATT_SCALE_FACTOR = PID_THR_BATT_SCALE_FACTOR * (1.0 + (batteryVoltageInVolts - PID_BATT_MIDDLE_VOLTAGE)*PID_BATT_VOLTAGE_SLOPE);
	}

	//implement mass compensation
	PID_THR_BATT_SCALE_FACTOR = (DRONE_WEIGHT / 1000.0) * PID_THR_BATT_SCALE_FACTOR;
}

void setupMotorPins()
{
	ledcSetup(M_FL_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_FR_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_BR_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_BL_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcAttachPin(PIN_M_FL, M_FL_CHANNEL);
	ledcAttachPin(PIN_M_FR, M_FR_CHANNEL);
	ledcAttachPin(PIN_M_BR, M_BR_CHANNEL);
	ledcAttachPin(PIN_M_BL, M_BL_CHANNEL);
}

void pwmMicroSeconds(int _pwm_channel, int _microseconds)
{
	ledcWrite(_pwm_channel, _microseconds*PWM_MICROSECONDS_TO_BITS);
}

bool initBarometer()
{
	uint32_t initTime = millis();
	
#ifdef BAROMETER_MS5611
	while (!barometer.begin(MS5611_HIGH_RES))
#endif
#ifdef BAROMETER_BMP180
	while (!barometer.begin())
#endif	
#ifdef BAROMETER_BMP280
	while(!barometer.begin())		
#endif
	{
		if (millis() - initTime > BAROMETER_INIT_THRESHOLD)
		{
			statusBaro = statusType_InitFail;
			Serial.println("Baro init failed");
			return false;
		}
		delay(200);
	}
	statusBaro = statusType_Normal;
	
	Serial.println("Baro init success");
	#ifdef BAROMETER_BMP280
	barometer.setOversampling(4);
	#endif

	// The pressure sensor returns abolute pressure, which varies with altitude.
	// To remove the effects of altitude, use the sealevel function and your current altitude.
	// This number is commonly used in weather reports.
	// Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
	// Result: p0 = sea-level compensated pressure in mb



	delay(100);
	return true;
}

void processBarometer()
{
#ifdef BAROMETER_MS5611
	//Barometer Data Processing

	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)10) == pdTRUE)
	{
		barometer.runProcess();
		xSemaphoreGive(xI2CSemaphore);
	}
	barometerTemp = barometer.readTemperature(true);
	barometerPress = barometer.readPressure(true);
	double value = barometer.getAltitude(barometerPress);
#endif
#ifdef BAROMETER_BMP180
	char status = 0;
	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)10) == pdTRUE)
	{
		status = barometer.startTemperature();
		xSemaphoreGive(xI2CSemaphore);
	}

	if (status != 0)
	{
		// Wait for the measurement to complete:
		delay(status);

		status = 0;
		if (xSemaphoreTake(xI2CSemaphore, (TickType_t)10) == pdTRUE)
		{
			status = barometer.getTemperature(barometerTemp);
			xSemaphoreGive(xI2CSemaphore);
		}

		if (status != 0)
		{
			status = 0;
			if (xSemaphoreTake(xI2CSemaphore, (TickType_t)10) == pdTRUE)
			{
				status = barometer.startPressure(2);
				xSemaphoreGive(xI2CSemaphore);
			}
			
			if (status != 0)
			{
				// Wait for the measurement to complete:
				delay(status);

				// Retrieve the completed pressure measurement:
				// Note that the measurement is stored in the variable P.
				// Note also that the function requires the previous temperature measurement (T).
				// (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
				// Function returns 1 if successful, 0 if failure.
				status = 0;
				if (xSemaphoreTake(xI2CSemaphore, (TickType_t)10) == pdTRUE)
				{
					status = barometer.getPressure(barometerPress, barometerTemp);
					xSemaphoreGive(xI2CSemaphore);
				}
				
				if (status != 0)
				{


					if (barometer_initial_measurement)
					{
						sealevelPress = barometer.sealevel(barometerPress, EXISTING_ALTITUDE); // we're at 1655 meters (Boulder, CO)
						barometer_initial_measurement = false;
					}

					computedAlt = barometer.altitude(barometerPress, sealevelPress);
					statusBaro = statusType_Normal;
				}
				else statusBaro = statusType_Fail;
			}
			else statusBaro = statusType_Fail;
		}
		else statusBaro = statusType_Fail;
	}
	else statusBaro = statusType_Fail;

	double value = computedAlt;
#endif	

#ifdef BAROMETER_BMP280
	double value = 0;
	double T = 0;
	double P = SEALEVEL_PRESS;
	char result=0;

	result = barometer.getTemperatureAndPressure(T, P);
	if (result != 0)
	{
		value = barometer.altitude(P, SEALEVEL_PRESS);
		barometerPress = P;
		barometerTemp = T;
		//Serial.print("T = \t"); Serial.print(T, 2); Serial.print(" degC\t");
		//Serial.print("P = \t"); Serial.print(P, 2); Serial.print(" mBar\t");
		//Serial.print("A = \t"); Serial.print(value, 2); Serial.println(" m");
	}
	else {
		Serial.println("Barometer Error 2");
	}
#endif


	if (!isnan(value))
	{
		barometerAlt = value;
		baroReady = true;
	}
	result = barometer.startMeasurment();

	if (result = 0) {
		Serial.println("Barometer Error 1");
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
		compass.setDataRate(HMC5883L_DATARATE_30HZ);
		// Set number of samples averaged
		compass.setSamples(HMC5883L_SAMPLES_1);
		// Check settings
		compass.setOffset(0, 0);

		statusCompass = statusType_Normal;
		Serial.println("Compass Init Success!");
		return true;
	}
	else
	{
		Serial.println("Compass Init Failed!");
		statusCompass = statusType_InitFail;
		return false;
	}


}

void processCompass()
{

	Vector compassRaw = compass.readRaw();
	compassRaw.YAxis = -1 * compassRaw.YAxis;
	compassRaw.ZAxis = -1 * compassRaw.ZAxis;

	//--------------------------------------------------------------------------------
	/// Code part that will be used for calibration operation

	//if (compassRaw.XAxis > compassHdgXmax) compassHdgXmax = compassRaw.XAxis;
	//if (compassRaw.XAxis < compassHdgXmin) compassHdgXmin = compassRaw.XAxis;

	//if (compassRaw.YAxis > compassHdgYmax) compassHdgYmax = compassRaw.YAxis;
	//if (compassRaw.YAxis < compassHdgYmin) compassHdgYmin = compassRaw.YAxis;

	//if (compassRaw.ZAxis > compassHdgZmax) compassHdgZmax = compassRaw.ZAxis;
	//if (compassRaw.ZAxis < compassHdgZmin) compassHdgZmin = compassRaw.ZAxis;

	//compassHdgXoffset = (compassHdgXmax + compassHdgXmin) / 2;
	//compassHdgYoffset = (compassHdgYmax + compassHdgYmin) / 2;
	//compassHdgZoffset = (compassHdgZmax + compassHdgZmin) / 2;

	//compassHdgXrange = (compassHdgXmax - compassHdgXmin);
	//compassHdgYrange = (compassHdgYmax - compassHdgYmin);
	//compassHdgZrange = (compassHdgZmax - compassHdgZmin);

	//Serial.print("xOffset:");
	//Serial.print(compassHdgXoffset);
	//Serial.print("  xRange:");
	//Serial.print(compassHdgXrange);
	//Serial.print("  yOffset:");
	//Serial.print(compassHdgYoffset);
	//Serial.print("  yRange:");
	//Serial.print(compassHdgYrange);
	//Serial.print("  zOffset:");
	//Serial.print(compassHdgZoffset);
	//Serial.print("  zRange:");
	//Serial.println(compassHdgZrange);

	// End of Calibration Part
	//--------------------------------------------------------------------------------


	//--------------------------------------------------------------------------------
	/// Code part that will be used for normal operation
	//--------------------------------------------------------------------------------
	/// Offset and scale values should be defined after the calibration process

	Vector compassNorm;

	compassNorm.XAxis = (compassRaw.XAxis - compassHdgXoffset) * (compassHdgYrange/1000.0) * (compassHdgZrange/1000.0);
	compassNorm.YAxis = (compassRaw.YAxis - compassHdgYoffset) * (compassHdgXrange / 1000.0) * (compassHdgZrange / 1000.0);
	compassNorm.ZAxis = (compassRaw.ZAxis - compassHdgZoffset) * (compassHdgXrange / 1000.0) * (compassHdgYrange / 1000.0);


	Vector compassRotated;

	compassRotated.XAxis = compassNorm.XAxis*cos(qc.euler.theta) + compassNorm.ZAxis*cos(qc.euler.phi)*sin(qc.euler.theta) + compassNorm.YAxis*sin(qc.euler.theta)*sin(qc.euler.phi);
	compassRotated.YAxis = compassNorm.YAxis*cos(qc.euler.phi) - compassNorm.ZAxis*sin(qc.euler.phi);
	//compassRotated.ZAxis = compassNorm.ZAxis*cos(qc.euler.theta)*cos(qc.euler.phi) - compassNorm.XAxis*sin(qc.euler.theta) + compassNorm.YAxis*cos(qc.euler.theta)*sin(qc.euler.phi);

	// To calculate heading in degrees. 0 degree indicates North
	compassHdg = -1 * atan2((compassRotated.YAxis), (compassRotated.XAxis)) * 180 / M_PI;


	// If Compass Angle is requested in 0-360 Range 
	// if (compassHdg < 0) compassHdg += 360;

	// End of Normal Part
	//--------------------------------------------------------------------------------

}

void initOTA()
{
	while (!wifi_connected)
	{
		delay(1000);
	}
	// Port defaults to 3232
	// ArduinoOTA.setPort(3232);

	// Hostname defaults to esp3232-[MAC]
	// ArduinoOTA.setHostname("myesp32");

	// No authentication by default
	// ArduinoOTA.setPassword("admin");

	// Password can be set with it's md5 value as well
	// MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
	// ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

	ArduinoOTA.onStart([]() {
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH)
			type = "sketch";
		else // U_SPIFFS
			type = "filesystem";

		// NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
		//SPIFFS.end();
		Serial.println("Start updating " + type);
	});
	ArduinoOTA.onEnd([]() {
		Serial.println("\nEnd");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
		else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
		else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
		else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
		else if (error == OTA_END_ERROR) Serial.println("End Failed");
	});
	ArduinoOTA.begin();
	Serial.print("OTA Ready,");
	Serial.print("  IP address: ");
#ifdef DRONE_AP
	Serial.println(WiFi.softAPIP());
#else
	Serial.println(WiFi.localIP());
#endif //DRONE_AP
}

void updateUdpMsgVars()
{
	/// !!! Do not forget to add multiplication to following identities with correct resolution values
	switch (MsgUdpT01.message.pidCommandState)
	{
	case pidCommandApplyRatePitchRoll:
		applyPidCommandRatePitchRoll();
		break;

	case pidCommandApplyAnglePitchRoll:
		applyPidCommandAnglePitchRoll();
		break;

	case pidCommandApplyRateYaw:
		applyPidCommandRateYaw();
		break;

	case pidCommandApplyAngleYaw:
		applyPidCommandAngleYaw();
		break;

	case pidCommandApplyAll:
		applyPidCommandRatePitchRoll();
		applyPidCommandAnglePitchRoll();
		applyPidCommandRateYaw();
		applyPidCommandAngleYaw();
		applyPidCommandPosAlt();
		applyPidCommandVelAlt();
		applyPidCommandAccAlt();
		applyPidCommandAccPos();
		break;

	case pidCommandApplyVelAlt:
		applyPidCommandVelAlt();
		break;

	case pidCommandApplyAccAlt:
		applyPidCommandAccAlt();
		break;

	case pidCommandApplyPosAlt:
		applyPidCommandPosAlt();
		break;

	case pidCommandApplyAccPos:
		applyPidCommandAccPos();
		break;
	default:break;
	}

	setHomePoint = MsgUdpT01.message.saveHomePos;
}

void updateAndroidUdpMsgVars()
{
	setHomePoint = MsgUdpTAndroid.message.saveHomePos;
}

void applyPidCommandRatePitchRoll()
{
	//Set Pitch Rate PID parameters
	pidVars.ratePitch.Kp = MsgUdpT01.message.pidRatePitchRollKp * RESOLUTION_PID_RATE_KP;
	pidVars.ratePitch.Ki = MsgUdpT01.message.pidRatePitchRollKi * RESOLUTION_PID_RATE_KI;
	pidVars.ratePitch.Kd = MsgUdpT01.message.pidRatePitchRollKd * RESOLUTION_PID_RATE_KD;
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);
	//Set Roll Rate PID parameters
	pidVars.rateRoll.Kp = pidVars.ratePitch.Kp;
	pidVars.rateRoll.Ki = pidVars.ratePitch.Ki;
	pidVars.rateRoll.Kd = pidVars.ratePitch.Kd;
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);
}

void applyPidCommandAnglePitchRoll()
{
	//Set Pitch Angle PID parameters
	pidVars.anglePitch.Kp = MsgUdpT01.message.pidAnglePitchRollKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.anglePitch.Ki = MsgUdpT01.message.pidAnglePitchRollKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.anglePitch.Kd = MsgUdpT01.message.pidAnglePitchRollKd * RESOLUTION_PID_ANGLE_KD;
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);
	//Set Roll Angle PID parameters
	pidVars.angleRoll.Kp = pidVars.anglePitch.Kp;
	pidVars.angleRoll.Ki = pidVars.anglePitch.Ki;
	pidVars.angleRoll.Kd = pidVars.anglePitch.Kd;
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);
}

void applyPidCommandRateYaw()
{
	//Set Yaw Rate PID parameters
	pidVars.rateYaw.Kp = MsgUdpT01.message.pidRateYawKp * RESOLUTION_PID_RATE_KP;
	pidVars.rateYaw.Ki = MsgUdpT01.message.pidRateYawKi * RESOLUTION_PID_RATE_KI;
	pidVars.rateYaw.Kd = MsgUdpT01.message.pidRateYawKd * RESOLUTION_PID_RATE_KD;
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);
}

void applyPidCommandAngleYaw()
{
	//Set Yaw Angle PID parameters
	pidVars.angleYaw.Kp = MsgUdpT01.message.pidAngleYawKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.angleYaw.Ki = MsgUdpT01.message.pidAngleYawKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.angleYaw.Kd = MsgUdpT01.message.pidAngleYawKd * RESOLUTION_PID_ANGLE_YAW_KD;
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);
}

void applyPidCommandPosAlt()
{
	//Set Altitude Posocity PID parameters
	pidVars.posAlt.Kp = MsgUdpT01.message.pidPosAltKp * RESOLUTION_PID_POS_KP;
	pidVars.posAlt.Ki = MsgUdpT01.message.pidPosAltKi * RESOLUTION_PID_POS_KI;
	pidVars.posAlt.Kd = MsgUdpT01.message.pidPosAltKd * RESOLUTION_PID_POS_KD;
	pidPosAlt.SetTunings(pidVars.posAlt.Kp, pidVars.posAlt.Ki, pidVars.posAlt.Kd);
}

void applyPidCommandVelAlt()
{
	//Set Altitude Velocity PID parameters
	//pidVars.velAlt.Kp = MsgUdpT01.message.pidVelAltKp * RESOLUTION_PID_VEL_KP;
	//pidVars.velAlt.Ki = MsgUdpT01.message.pidVelAltKi * RESOLUTION_PID_VEL_KI;
	//pidVars.velAlt.Kd = MsgUdpT01.message.pidVelAltKd * RESOLUTION_PID_VEL_KD;
	//pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);

	pidVars.velX.Kp = MsgUdpT01.message.pidVelAltKp * RESOLUTION_PID_VEL_KP;
	pidVars.velX.Ki = MsgUdpT01.message.pidVelAltKi * RESOLUTION_PID_VEL_KI;
	pidVars.velX.Kd = MsgUdpT01.message.pidVelAltKd * RESOLUTION_PID_VEL_KD;
	pidVelX.SetTunings(pidVars.velX.Kp, pidVars.velX.Ki, pidVars.velX.Kd);
	pidVelY.SetTunings(pidVars.velX.Kp, pidVars.velX.Ki, pidVars.velX.Kd);
}

void applyPidCommandAccAlt()
{
	//Set Altitude Acc PID parameters
	pidVars.accAlt.Kp = MsgUdpT01.message.pidAccAltKp * RESOLUTION_PID_ACC_KP;
	pidVars.accAlt.Ki = MsgUdpT01.message.pidAccAltKi * RESOLUTION_PID_ACC_KI;
	pidVars.accAlt.Kd = MsgUdpT01.message.pidAccAltKd * RESOLUTION_PID_ACC_KD;
	pidAccAlt.SetTunings(pidVars.accAlt.Kp * DRONE_WEIGHT / 1000.0, pidVars.accAlt.Ki * DRONE_WEIGHT / 1000.0, pidVars.accAlt.Kd * DRONE_WEIGHT / 1000.0);
}

void applyPidCommandAccPos()
{
	//Set Position Acc PID parameters
	pidVars.accX.Kp = MsgUdpT01.message.pidAccPosKp * RESOLUTION_PID_ACC_KP;
	pidVars.accX.Ki = MsgUdpT01.message.pidAccPosKi * RESOLUTION_PID_ACC_KI;
	pidVars.accX.Kd = MsgUdpT01.message.pidAccPosKd * RESOLUTION_PID_ACC_KD;
	pidAccX.SetTunings(pidVars.accX.Kp, pidVars.accX.Ki, pidVars.accX.Kd);

	pidVars.accY.Kp = MsgUdpT01.message.pidAccPosKp * RESOLUTION_PID_ACC_KP;
	pidVars.accY.Ki = MsgUdpT01.message.pidAccPosKi * RESOLUTION_PID_ACC_KI;
	pidVars.accY.Kd = MsgUdpT01.message.pidAccPosKd * RESOLUTION_PID_ACC_KD;
	pidAccY.SetTunings(pidVars.accY.Kp, pidVars.accY.Ki, pidVars.accY.Kd);
}

void prepareUDPmessages()
{
	MsgUdpR01.message.timeStamp				= millis();
	MsgUdpR01.message.statusMpu				= statusMpu;
	MsgUdpR01.message.statusBaro			= statusBaro;
	MsgUdpR01.message.statusCompass			= statusCompass;
	MsgUdpR01.message.statusUdp				= statusUdp;
	MsgUdpR01.message.statusGS				= statusGS;
	MsgUdpR01.message.mpuGyroX				= qc.gyro.x;
	MsgUdpR01.message.mpuGyroY				= qc.gyro.y;
	MsgUdpR01.message.mpuGyroZ				= qc.gyro.z;
	MsgUdpR01.message.mpuAccX				= qc.accel.x;
	MsgUdpR01.message.mpuAccY				= qc.accel.y;
	MsgUdpR01.message.mpuAccZ				= qc.accel.z;
	MsgUdpR01.message.mpuAccWorldX			= qc.accelWorld.x;
	MsgUdpR01.message.mpuAccWorldY			= qc.accelWorld.y;
	MsgUdpR01.message.mpuAccWorldZ			= qc.accelWorld.z;
	MsgUdpR01.message.mpuYaw				= qc.euler.psi * 180 / M_PI;
	MsgUdpR01.message.mpuPitch				= qc.euler.theta * 180 / M_PI;
	MsgUdpR01.message.mpuRoll				= qc.euler.phi * 180 / M_PI;
	MsgUdpR01.message.baroTemp				= barometerTemp;
	MsgUdpR01.message.baroAlt				= barometerAlt;
	MsgUdpR01.message.ultrasonicDist		= ultrasonicDistanceFiltered;
	MsgUdpR01.message.compassHdg			= compassHdgEstimated;
	MsgUdpR01.message.batteryVoltage    	= batteryVoltageInVolts;
	MsgUdpR01.message.quadAccelerationWorldX = qc.accelWorldEstimated.x;
	MsgUdpR01.message.quadVelocityWorldX	= qc.velWorldEstimated.x;
	MsgUdpR01.message.quadPositionWorldX	= qc.posWorldEstimated.x;
	MsgUdpR01.message.quadAccelerationWorldY = qc.accelWorldEstimated.y;
	MsgUdpR01.message.quadVelocityWorldY	= qc.velWorldEstimated.y;
	MsgUdpR01.message.quadPositionWorldY	= qc.posWorldEstimated.y;
	MsgUdpR01.message.quadAccelerationWorldZ = qc.accelWorldEstimated.z;
	MsgUdpR01.message.quadVelocityWorldZ	= qc.velWorldEstimated.z;
	MsgUdpR01.message.quadPositionWorldZ	= qc.posWorldEstimated.z;
	MsgUdpR01.message.modeQuad				= modeQuad;
	MsgUdpR01.message.autoModeStatus		= autoModeStatus;
	MsgUdpR01.message.statusRx				= statusRx;
	MsgUdpR01.message.rxThrottle			= cmdRxThr;
	MsgUdpR01.message.rxPitch				= cmdRxPitchCalibrated;
	MsgUdpR01.message.rxRoll				= cmdRxRollCalibrated;
	MsgUdpR01.message.rxYaw					= cmdRxYaw;
	MsgUdpR01.message.rx6thCh				= cmdRx6thCh;

	MsgUdpR01.message.pidRatePitchKp		= pidVars.ratePitch.Kp / RESOLUTION_PID_RATE_KP;
	MsgUdpR01.message.pidRatePitchKi		= pidVars.ratePitch.Ki / RESOLUTION_PID_RATE_KI;
	MsgUdpR01.message.pidRatePitchKd		= pidVars.ratePitch.Kd / RESOLUTION_PID_RATE_KD;
	MsgUdpR01.message.pidRatePitchOutput	= pidVars.ratePitch.output;
	MsgUdpR01.message.pidRatePitchPresult	= pidRatePitch.Get_P_Result();
	MsgUdpR01.message.pidRatePitchIresult	= pidRatePitch.Get_I_Result();
	MsgUdpR01.message.pidRatePitchDresult	= pidRatePitch.Get_D_Result();
	MsgUdpR01.message.pidAnglePitchKp		= pidVars.anglePitch.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgUdpR01.message.pidAnglePitchKi		= pidVars.anglePitch.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgUdpR01.message.pidAnglePitchKd		= pidVars.anglePitch.Kd / RESOLUTION_PID_ANGLE_KD;
	MsgUdpR01.message.pidAnglePitchOutput	= pidVars.anglePitch.output;
	MsgUdpR01.message.pidAnglePitchPresult	= pidAnglePitch.Get_P_Result();
	MsgUdpR01.message.pidAnglePitchIresult	= pidAnglePitch.Get_I_Result();
	MsgUdpR01.message.pidAnglePitchDresult	= pidAnglePitch.Get_D_Result();

	MsgUdpR01.message.pidRateYawKp			= pidVars.rateYaw.Kp / RESOLUTION_PID_RATE_KP;
	MsgUdpR01.message.pidRateYawKi			= pidVars.rateYaw.Ki / RESOLUTION_PID_RATE_KI;
	MsgUdpR01.message.pidRateYawKd			= pidVars.rateYaw.Kd / RESOLUTION_PID_RATE_KD;
	MsgUdpR01.message.pidRateYawOutput		= pidVars.rateYaw.output;
	MsgUdpR01.message.pidRateYawPresult		= pidRateYaw.Get_P_Result();
	MsgUdpR01.message.pidRateYawIresult		= pidRateYaw.Get_I_Result();
	MsgUdpR01.message.pidRateYawDresult		= pidRateYaw.Get_D_Result();
	MsgUdpR01.message.pidAngleYawKp			= pidVars.angleYaw.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgUdpR01.message.pidAngleYawKi			= pidVars.angleYaw.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgUdpR01.message.pidAngleYawKd			= pidVars.angleYaw.Kd / RESOLUTION_PID_ANGLE_YAW_KD;
	MsgUdpR01.message.pidAngleYawOutput		= pidVars.angleYaw.output;
	MsgUdpR01.message.pidAngleYawPresult	= pidAngleYaw.Get_P_Result();
	MsgUdpR01.message.pidAngleYawIresult	= pidAngleYaw.Get_I_Result();
	MsgUdpR01.message.pidAngleYawDresult	= pidAngleYaw.Get_D_Result();
	MsgUdpR01.message.commandedYawAngle		= cmdMotorYaw;

	MsgUdpR01.message.pidPosAltKp			= pidVars.posAlt.Kp / RESOLUTION_PID_POS_KP;
	MsgUdpR01.message.pidPosAltKi			= pidVars.posAlt.Ki / RESOLUTION_PID_POS_KI;
	MsgUdpR01.message.pidPosAltKd			= pidVars.posAlt.Kd / RESOLUTION_PID_POS_KD;
	MsgUdpR01.message.pidPosAltOutput		= pidVars.posAlt.output;
	MsgUdpR01.message.pidPosAltPresult		= pidPosAlt.Get_P_Result();
	MsgUdpR01.message.pidPosAltIresult		= pidPosAlt.Get_I_Result();
	MsgUdpR01.message.pidPosAltDresult		= pidPosAlt.Get_D_Result();

	MsgUdpR01.message.pidVelAltKp			= pidVars.velAlt.Kp / RESOLUTION_PID_VEL_KP;
	MsgUdpR01.message.pidVelAltKi			= pidVars.velAlt.Ki / RESOLUTION_PID_VEL_KI;
	MsgUdpR01.message.pidVelAltKd			= pidVars.velAlt.Kd / RESOLUTION_PID_VEL_KD;
	MsgUdpR01.message.pidVelAltOutput		= pidVars.velAlt.output;
	MsgUdpR01.message.pidVelAltPresult		= pidVelAlt.Get_P_Result();
	MsgUdpR01.message.pidVelAltIresult		= pidVelAlt.Get_I_Result();
	MsgUdpR01.message.pidVelAltDresult		= pidVelAlt.Get_D_Result();

	MsgUdpR01.message.pidAccAltKp			= pidVars.accAlt.Kp / RESOLUTION_PID_ACC_KP;
	MsgUdpR01.message.pidAccAltKi			= pidVars.accAlt.Ki / RESOLUTION_PID_ACC_KI;
	MsgUdpR01.message.pidAccAltKd			= pidVars.accAlt.Kd / RESOLUTION_PID_ACC_KD;
	MsgUdpR01.message.pidAccAltOutput		= pidVars.accAlt.output;
	MsgUdpR01.message.pidAccAltPresult		= pidAccAlt.Get_P_Result();
	MsgUdpR01.message.pidAccAltIresult		= pidAccAlt.Get_I_Result();
	MsgUdpR01.message.pidAccAltDresult		= pidAccAlt.Get_D_Result();

	MsgUdpR01.message.pidPosXKp				= pidVars.posX.Kp / RESOLUTION_PID_POS_KP;
	MsgUdpR01.message.pidPosXKi				= pidVars.posX.Ki / RESOLUTION_PID_POS_KI;
	MsgUdpR01.message.pidPosXKd				= pidVars.posX.Kd / RESOLUTION_PID_POS_KD;
	MsgUdpR01.message.pidPosXOutput			= pidVars.posX.output;
	MsgUdpR01.message.pidPosXPresult		= pidPosX.Get_P_Result();
	MsgUdpR01.message.pidPosXIresult		= pidPosX.Get_I_Result();
	MsgUdpR01.message.pidPosXDresult		= pidPosX.Get_D_Result();

	MsgUdpR01.message.pidVelXKp				= pidVars.velX.Kp / RESOLUTION_PID_VEL_KP;
	MsgUdpR01.message.pidVelXKi				= pidVars.velX.Ki / RESOLUTION_PID_VEL_KI;
	MsgUdpR01.message.pidVelXKd				= pidVars.velX.Kd / RESOLUTION_PID_VEL_KD;
	MsgUdpR01.message.pidVelXOutput			= pidVars.velX.output;
	MsgUdpR01.message.pidVelXPresult		= pidVelX.Get_P_Result();
	MsgUdpR01.message.pidVelXIresult		= pidVelX.Get_I_Result();
	MsgUdpR01.message.pidVelXDresult		= pidVelX.Get_D_Result();

	MsgUdpR01.message.pidAccXKp				= pidVars.accX.Kp / RESOLUTION_PID_ACC_KP;
	MsgUdpR01.message.pidAccXKi				= pidVars.accX.Ki / RESOLUTION_PID_ACC_KI;
	MsgUdpR01.message.pidAccXKd				= pidVars.accX.Kd / RESOLUTION_PID_ACC_KD;
	MsgUdpR01.message.pidAccXOutput			= pidVars.accX.output;
	MsgUdpR01.message.pidAccXPresult		= pidAccX.Get_P_Result();
	MsgUdpR01.message.pidAccXIresult		= pidAccX.Get_I_Result();
	MsgUdpR01.message.pidAccXDresult		= pidAccX.Get_D_Result();

	MsgUdpR01.message.gpsStatus				= qcGPS.gpsIsFix;
	MsgUdpR01.message.gpsLat				= qcGPS.lat*1e7;
	MsgUdpR01.message.gpsLon				= qcGPS.lon*1e7;
	MsgUdpR01.message.gpsAlt				= qcGPS.alt;
	MsgUdpR01.message.homeLat				= homePoint.lat*1e7;
	MsgUdpR01.message.homeLon				= homePoint.lon*1e7;
	MsgUdpR01.message.homeAlt				= homePoint.alt;
	MsgUdpR01.message.gpsVelN				= qcGPS.nedVelocity.N;
	MsgUdpR01.message.gpsVelE				= qcGPS.nedVelocity.E;
	MsgUdpR01.message.gpsVelD				= qcGPS.nedVelocity.D;
	MsgUdpR01.message.gpsPosAccuracy		= qcGPS.posAccuracy;
	MsgUdpR01.message.gpsVelAccuracy		= qcGPS.velAccuracy;

	MsgUdpR01.message.posHoldAvailable		= posHoldAvailable;
	MsgUdpR01.message.velHoldAvailable		= velHoldAvailable;

	MsgUdpR01.message.lidar_distance		= lidar_distance;

	MsgUdpR01.getPacket();
}

void prepareAndroidUDPmessages()
{
	MsgUdpRAndroid.message.timeStamp = millis();

	MsgUdpRAndroid.message.modeQuad = modeQuad;
	MsgUdpRAndroid.message.autoModeStatus = qcGPS.gpsIsFix; // autoModeStatus; // qcGPS.gpsIsFix is being used temporarily
	MsgUdpRAndroid.message.homePointSelected = homePointSelected;
	MsgUdpRAndroid.message.posHoldAvailable = posHoldAvailable;

	MsgUdpRAndroid.message.batteryVoltage = batteryVoltageInVolts;

	MsgUdpRAndroid.message.mpuRoll = qc.euler.phi * 180 / M_PI;
	MsgUdpRAndroid.message.mpuPitch = qc.euler.theta * 180 / M_PI;
	MsgUdpRAndroid.message.mpuYaw = qc.euler.psi * 180 / M_PI;

	MsgUdpRAndroid.message.baroAlt = barometerAlt;

	MsgUdpRAndroid.message.compassHdg = compassHdgEstimated;

	MsgUdpRAndroid.message.gpsLat = qcGPS.lat*1e7;
	MsgUdpRAndroid.message.gpsLon = qcGPS.lon*1e7;
	MsgUdpRAndroid.message.gpsAlt = qcGPS.alt;
	MsgUdpRAndroid.message.gpsVelN = qcGPS.nedVelocity.N;
	MsgUdpRAndroid.message.gpsVelE = qcGPS.nedVelocity.E;
	MsgUdpRAndroid.message.gpsPosAccuracy = qcGPS.posAccuracy;
	MsgUdpRAndroid.message.gpsVelAccuracy = qcGPS.velAccuracy;

	MsgUdpRAndroid.message.quadVelocityWorldX = qc.velWorldEstimated.x;
	MsgUdpRAndroid.message.quadPositionWorldX = qc.posWorldEstimated.x;
	MsgUdpRAndroid.message.quadVelocityWorldY = qc.velWorldEstimated.y;
	MsgUdpRAndroid.message.quadPositionWorldY = qc.posWorldEstimated.y;
	MsgUdpRAndroid.message.quadVelocityWorldZ = qc.velWorldEstimated.z;
	MsgUdpRAndroid.message.quadPositionWorldZ = qc.posWorldEstimated.z;

	MsgUdpRAndroid.getPacket();
}

void checkMpuGSHealth()
{
	if ((millis() - mpuLastDataTime> MPU_DATATIME_THRESHOLD) && statusMpu != statusType_NotInitiated)
	{
		statusMpu = statusType_Fail;
		//try to restart MPU and check mpu status there again
		initMPU();
	}


	if (millis() - udpLastMessageTime > GS_CON_LOST_THRESHOLD)
	{
		statusGS = statusType_Fail;
	}
	else
	{
		statusGS = statusType_Normal;
	}
}

void connectToWiFi()
{

#ifdef DRONE_AP

	Serial.println();
	Serial.print("WiFi starting as access point:");
	Serial.println(DRONE_AP_NAME);
	WiFi.softAP(DRONE_AP_NAME, DRONE_AP_PASS);

	Serial.println("IP address: ");
	Serial.println(WiFi.softAPIP());


	wifi_connected = true;
	connectUdp();

#else
	// delete old config
	WiFi.disconnect(true);
	//register event handler
	WiFi.onEvent(WiFiEvent);

	Serial.print("Connecting to ");
	Serial.println(WIFI_SSID);

	WiFi.begin(WIFI_SSID, WIFI_PASS);
	unsigned long startTimeWifi = millis();
	while (WiFi.status() != WL_CONNECTED) {
		delay(500);
		Serial.print(".");
		if (millis() - startTimeWifi > WIFI_CONNECTION_THRESHOLD)
		{
			Serial.println("Wifi connection timed out!");
			break;
		}
	}

	if (WiFi.status() == WL_CONNECTED)
	{

		wifi_connected = true;
		Serial.println("");
		Serial.println("WiFi connected");
		Serial.print("IP address: ");
		Serial.println(WiFi.localIP());
		connectUdp();

	}
	else
	{
		wifi_connected = false;
		Serial.println("Wifi could not be connected!");
	}
#endif // DRONE_AP
}

void connectUdp()
{

#ifdef DRONE_AP
	if (udp.begin(WiFi.softAPIP(), UDP_PORT) == 1)
	{
		udp.clearWriteError();
		udp_connected = true;
		Serial.print("UDP Started on IP:");
		Serial.print(WiFi.softAPIP());
		Serial.print("@");
		Serial.println(UDP_PORT);
	}
	else
	{
		udp_connected = false;
		Serial.println("UDP could not be started!");
	}
#else
	if (udp.begin(WiFi.localIP(), UDP_PORT) == 1)
	{
		udp.clearWriteError();
		udp_connected = true;
		Serial.print("UDP Started on IP:");
		Serial.print(WiFi.localIP());
		Serial.print("@");
		Serial.println(UDP_PORT);
	}
	else
	{
		udp_connected = false;
		Serial.println("UDP could not be started!");
	}
#endif // DRONE_AP
}

void WiFiEvent(WiFiEvent_t event) {
	switch (event) {
	case SYSTEM_EVENT_STA_GOT_IP:
		//When connected set 
		Serial.print("Event Handler: WiFi connected! IP address: ");
		Serial.println(WiFi.localIP());
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		Serial.println("Event Handler: WiFi lost connection");
		//connectToWiFi();
		break;
	}
}

//void filterAccXYPIDoutputs() // This filter is not necessary
//{
//	// Filtering AccX and AccY PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.accX.outputFiltered = filtObjAccPIDoutX.filter(pidVars.accX.output, 0);
//	pidVars.accY.outputFiltered = filtObjAccPIDoutY.filter(pidVars.accY.output, 0);
//
//}

//void filterAnglePIDoutputs() // This filter is not necessary
//{
//	// Filtering Angle PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.angleRoll.outputFiltered = filtObjAnglePIDoutX.filter(pidVars.angleRoll.output, 0);
//	pidVars.anglePitch.outputFiltered = filtObjAnglePIDoutY.filter(pidVars.anglePitch.output, 0);
//	pidVars.angleYaw.outputFiltered = filtObjAnglePIDoutZ.filter(pidVars.angleYaw.output, 0);
//
//	//Serial.println(pidVars.angleYaw.output);
//}

void calculateGyroDifferentials()
{
	// Differantiate Gyro Values for Rate PID Kd branch
	// Calculate Filter Output
	qc.gyroDiff.x = filtObjGyroDiffX.filter(qc.gyro.x, DELTATIME_GYRO_DIFF);
	qc.gyroDiff.y = filtObjGyroDiffY.filter(qc.gyro.y, DELTATIME_GYRO_DIFF);
	qc.gyroDiff.z = filtObjGyroDiffZ.filter(qc.gyro.z, DELTATIME_GYRO_DIFF);

}

void calculateRateCmdDifferentials()
{
	// Differantiate Rate Commands for Rate PID Kd branch
	// Calculate Filter Output
	rateCmdDiff.x = filtObjRateCmdDiffX.filter(rateCmd.x, DELTATIME_RATECMD_DIFF);
	rateCmdDiff.y = filtObjRateCmdDiffY.filter(rateCmd.y, DELTATIME_RATECMD_DIFF);
	rateCmdDiff.z = filtObjRateCmdDiffZ.filter(rateCmd.z, DELTATIME_RATECMD_DIFF);

}

void calculateAngleCmdDifferentials()
{
	// Differantiate Angle Commands for Angle PID Kd branch
	// Calculate Filter Output
	angleCmdDiff.x = filtObjAngleCmdDiffX.filter(angleCmd.x, DELTATIME_ANGLECMD_DIFF);
	angleCmdDiff.y = filtObjAngleCmdDiffY.filter(angleCmd.y, DELTATIME_ANGLECMD_DIFF);

}

void calculateAccelWorldXYDifferentials()
{
	// Differantiate AccelWorld Values for Acc PID Kd branch
	// Calculate Filter Output
	qc.accelWorldDiff.x = filtObjAccelWorldDiffX.filter(qc.accelWorldEstimated.x, DELTATIME_ACCELWORLD_DIFF);
	qc.accelWorldDiff.y = filtObjAccelWorldDiffY.filter(qc.accelWorldEstimated.y, DELTATIME_ACCELWORLD_DIFF);

}

void calculateAccelWorldZDifferentials()
{
	// Differantiate AccelWorld Values for Acc PID Kd branch
	// Calculate Filter Output
	qc.accelWorldDiff.z = filtObjAccelWorldDiffZ.filter(qc.accelWorldEstimated.z, DELTATIME_ACCELWORLD_DIFF);

}

//void filterPosAltPIDoutputs() // This filter is not necessary
//{
//	// Filtering Pos Altitude PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.posAlt.outputFiltered = filtObjPosPIDoutZ.filter(pidVars.posAlt.output, 0);
//}

//void filterPosXYPIDoutputs() // This filter is not necessary
//{
//	// Filtering Position X and Y PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.posX.outputFiltered = filtObjPosPIDoutX.filter(pidVars.posX.output, 0);
//	pidVars.posY.outputFiltered = filtObjPosPIDoutY.filter(pidVars.posY.output, 0);
//}

//void filterVelXYPIDoutputs() // This filter is not necessary
//{
//	// Filtering Velocity X and Y PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.velX.outputFiltered = filtObjVelPIDoutX.filter(pidVars.velX.output, 0);
//	pidVars.velY.outputFiltered = filtObjVelPIDoutY.filter(pidVars.velY.output, 0);
//}

//void filterVelAltPIDoutputs() // This filter is not necessary
//{
//	// Filtering Velocity Altitude PID Outputs by LPF
//	// Calculate Filter Output
//	pidVars.velAlt.outputFiltered = filtObjVelPIDoutZ.filter(pidVars.velAlt.output, 0);
//}

void calculateAccelCmdXYDifferentials()
{
	//// Differantiate Acceleration Commands for Acceleration PID Kd branch ////
	// Calculate Filter Output
	accelCmdDiff.x = filtObjAccelCmdDiffX.filter(accelCmd.x, DELTATIME_ACCELCMD_DIFF);
	accelCmdDiff.y = filtObjAccelCmdDiffY.filter(accelCmd.y, DELTATIME_ACCELCMD_DIFF);
}

void calculateAccelCmdZDifferentials()
{
	//// Differantiate Acceleration Commands for Acceleration PID Kd branch ////
	// Calculate Filter Output
	accelCmdDiff.z = filtObjAccelCmdDiffZ.filter(accelCmd.z, DELTATIME_ACCELCMD_DIFF);
}

void calculatePosCmdXYDifferentials()
{
	//// Differantiate Position Commands for Position PID Kd branch ////
	// Calculate Filter Output
	posCmdDiff.x = filtObjPosCmdDiffX.filter(posCmd.x, DELTATIME_POSCMD_DIFF);
	posCmdDiff.y = filtObjPosCmdDiffY.filter(posCmd.y, DELTATIME_POSCMD_DIFF);
}

void calculatePosCmdZDifferentials()
{
	//// Differantiate Position Commands for Altitude PID Kd branch ////
	// Calculate Filter Output
	posCmdDiff.z = filtObjPosCmdDiffZ.filter(posCmd.z, DELTATIME_POSCMD_DIFF);
}

void calculateVelCmdXYDifferentials()
{
	//// Differantiate Velocity Commands for Velocity PID Kd branch ////
	// Calculate Filter Output
	velCmdDiff.x = filtObjVelCmdDiffX.filter(velCmd.x, DELTATIME_VELCMD_DIFF);
	velCmdDiff.y = filtObjVelCmdDiffY.filter(velCmd.y, DELTATIME_VELCMD_DIFF);
}

void calculateVelCmdZDifferentials()
{
	//// Differantiate Velocity Commands for Velocity PID Kd branch ////
	// Calculate Filter Output
	velCmdDiff.z = filtObjVelCmdDiffZ.filter(velCmd.z, DELTATIME_VELCMD_DIFF);
}

double sign_sqrt(double _var)
{
	if (_var < 0) return -sqrt(-_var);
	else if (_var > 0) return sqrt(_var);
	else return 0;
}

void calculateGeodetic2Ecef(double _lat, double _lon, double _alt, double *_ecefX, double *_ecefY, double *_ecefZ)
{
	double nE;
	nE = REA_SEMI_MAJOR_AXIS / sqrt(1 - pow(ECCENTRICITY, 2) * pow(sin(_lat * M_PI / 180), 2));
	
	*_ecefX = (nE + _alt) * cos(_lat * M_PI / 180) * cos(_lon * M_PI / 180);
	*_ecefY = (nE + _alt) * cos(_lat * M_PI / 180) * sin(_lon * M_PI / 180);
	*_ecefZ = (nE * (1 - pow(ECCENTRICITY, 2)) + _alt) * sin(_lat * M_PI / 180);

}

void calculateEcef2Ned(double _ecefX, double _ecefY, double _ecefZ, double _ecefRefX, double _ecefRefY, double _ecefRefZ, double _latRef, double _lonRef, double *_nedX, double *_nedY, double *_nedZ)
{	
	double rotEcef2Ned[9];
	rotEcef2Ned[0] = -sin(_latRef * M_PI / 180) * cos(_lonRef * M_PI / 180);
	rotEcef2Ned[1] = -sin(_lonRef * M_PI / 180);
	rotEcef2Ned[2] = -cos(_latRef * M_PI / 180) * cos(_lonRef * M_PI / 180);
	rotEcef2Ned[3] = -sin(_latRef * M_PI / 180) * sin(_lonRef * M_PI / 180);
	rotEcef2Ned[4] = cos(_lonRef * M_PI / 180);
	rotEcef2Ned[5] = -cos(_latRef * M_PI / 180) * sin(_lonRef * M_PI / 180);
	rotEcef2Ned[6] = cos(_latRef * M_PI / 180);
	rotEcef2Ned[7] = 0.0;
	rotEcef2Ned[8] = -sin(_latRef * M_PI / 180);

	*_nedX = (_ecefX - _ecefRefX) * rotEcef2Ned[0] + (_ecefY - _ecefRefY) * rotEcef2Ned[3] + (_ecefZ - _ecefRefZ) * rotEcef2Ned[6];
	*_nedY = (_ecefX - _ecefRefX) * rotEcef2Ned[1] + (_ecefY - _ecefRefY) * rotEcef2Ned[4] + (_ecefZ - _ecefRefZ) * rotEcef2Ned[7];
	*_nedZ = (_ecefX - _ecefRefX) * rotEcef2Ned[2] + (_ecefY - _ecefRefY) * rotEcef2Ned[5] + (_ecefZ - _ecefRefZ) * rotEcef2Ned[8];
}

void convertNed2World(double _nedN, double _nedE, double _heading, double *_worldX, double *_worldY)
{
	*_worldX = _nedN * cos(_heading * M_PI / 180) + _nedE * sin(_heading * M_PI / 180);
	*_worldY = _nedN * -sin(_heading * M_PI / 180) + _nedE * cos(_heading * M_PI / 180);
}

void convertWorld2Ned(double _worldX, double _worldY, double _heading, double *_nedN, double *_nedE)
{
	*_nedN = _worldX * cos(_heading * M_PI / 180) + _worldY * -sin(_heading * M_PI / 180);
	*_nedE = _worldX * sin(_heading * M_PI / 180) + _worldY * cos(_heading * M_PI / 180);
}

void initSDcard()
{
	if (!SD.begin(PIN_SDCARD_CS, SPI, 40000000U, "/sd")) {
		Serial.println("Card Mount Failed");
		return;
	}
	uint8_t cardType = SD.cardType();

	if (cardType == CARD_NONE) {
		Serial.println("No SD card attached");
		return;
	}

	Serial.print("SD Card Type: ");
	if (cardType == CARD_MMC) {
		Serial.println("MMC");
	}
	else if (cardType == CARD_SD) {
		Serial.println("SDSC");
	}
	else if (cardType == CARD_SDHC) {
		Serial.println("SDHC");
	}
	else {
		Serial.println("UNKNOWN");
	}

	uint64_t cardSize = SD.cardSize() / (1024 * 1024);
	Serial.printf("SD Card Size: %lluMB\n", cardSize);
}

double getAltVelCmd(double _val)
{
	if (_val < ALT_VEL_ZERO_CMD_MIN) return (_val - ALT_VEL_ZERO_CMD_MIN);
	else if (_val > ALT_VEL_ZERO_CMD_MAX) return (_val - ALT_VEL_ZERO_CMD_MAX);
	else  return 0;
}