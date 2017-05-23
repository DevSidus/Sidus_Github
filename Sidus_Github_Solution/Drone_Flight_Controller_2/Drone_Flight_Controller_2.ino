/*
Name:		Drone_Flight_Controller.ino
Created:	11/10/2016 8:56:34 PM
Author:	SIDUS
Description: This is the main code for Drone_Flight_Controller Project
*/

//Global Include Files
#include <esp32-hal-ledc.h>
#include <Wire.h>
#include "rmt.h"

//Local Include Files
#include "cBuzzerMelody.h"
#include "Local_I2Cdev.h"
#include "Local_MPU6050_6Axis_MotionApps20.h"
#include "Local_MS5611.h"
#include "Local_HMC5883L.h"
#include "Local_PID_v1.h"
#include "PID_YawAngle.h"
#include "PID_AccAlt.h"
#include "Config.h"


//Global Class Definitions
MPU6050 mpu;
PID pidRatePitch(&pidVars.ratePitch.sensedVal, &pidVars.ratePitch.output, &pidVars.ratePitch.setpoint);
PID pidAnglePitch(&pidVars.anglePitch.sensedVal, &pidVars.anglePitch.output, &pidVars.anglePitch.setpoint, &pidVars.anglePitch.d_bypass);
PID pidRateRoll(&pidVars.rateRoll.sensedVal, &pidVars.rateRoll.output, &pidVars.rateRoll.setpoint);
PID pidAngleRoll(&pidVars.angleRoll.sensedVal, &pidVars.angleRoll.output, &pidVars.angleRoll.setpoint, &pidVars.angleRoll.d_bypass);
PID pidRateYaw(&pidVars.rateYaw.sensedVal, &pidVars.rateYaw.output, &pidVars.rateYaw.setpoint);
PID_YawAngle pidAngleYaw(&pidVars.angleYaw.sensedVal, &pidVars.angleYaw.output, &pidVars.angleYaw.setpoint, &pidVars.angleYaw.d_bypass);

PID pidVelAlt(&pidVars.velAlt.sensedVal, &pidVars.velAlt.output, &pidVars.velAlt.setpoint, &pidVars.velAlt.d_bypass);
PID_AccAlt pidAccAlt(&pidVars.accAlt.sensedVal, &pidVars.accAlt.output, &pidVars.accAlt.setpoint);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(SERIAL_COM_SPEED);

	Serial.println("Serial started");

	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_RX_ROLL, INPUT);
	pinMode(PIN_RX_PITCH, INPUT);
	pinMode(PIN_RX_THR, INPUT);
	pinMode(PIN_RX_YAW, INPUT);
	pinMode(PIN_RX_5TH_CHAN, INPUT);
	pinMode(PIN_RX_6TH_CHAN, INPUT);

	pinMode(PIN_BUZZER, OUTPUT);

	pinMode(PIN_MPU_POWER_ON, OUTPUT);

	digitalWrite(PIN_MPU_POWER_ON, LOW);
	delay(100);


	Wire.begin(PIN_MCU_SDA, PIN_MCU_SCL);
	Wire.setClock(800000L);


	xTaskCreatePinnedToCore(task_test, "task_test", 1024, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(task_mpu, "task_mpu", 10000, NULL, 20, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_0, "task_rx_0", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_1, "task_rx_1", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_2, "task_rx_2", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_3, "task_rx_3", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_4, "task_rx_4", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_5, "task_rx_5", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_mapCmd, "task_mapCmd", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_chkMode, "task_chkMode", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_PID, "task_PID", 8192, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_Motor, "task_Motor", 2048, NULL, 10, NULL, 0);

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
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
	}
	vTaskDelete(NULL);
}

void task_mpu(void * parameter)
{
	initMPU();
	while (true)
	{
		//mpuProcessStartTime = micros();
		processMpu();
		//mpuProcessTaskDuration = micros() - mpuProcessStartTime;
		//Serial.println(mpuProcessTaskDuration);
		delay(4);
	}
	vTaskDelete(NULL);
	return;
}

void task_rx_0(void * parameter)
{
	gpio_set_pull_mode(gpio_num_t(PIN_RX_ROLL), GPIO_PULLDOWN_ONLY);

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
	rmt_get_ringbuf_handler(rmt_rx_ch0.channel, &rb_ch0);
	rmt_rx_start(rmt_rx_ch0.channel, true);
	//--------RMT CONFIG RADIO ROLL CHANNEL-----------------//

	size_t rx_size = 0;

	while (true) 
	{
		//------------RMT GET RADIO ROLL PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch0 = (rmt_item32_t*)xRingbufferReceive(rb_ch0, &rx_size, RMT_RX_WAIT_TICKS);
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
	rmt_get_ringbuf_handler(rmt_rx_ch1.channel, &rb_ch1);
	rmt_rx_start(rmt_rx_ch1.channel, true);
	//--------RMT CONFIG RADIO PITCH CHANNEL-----------------//
	size_t rx_size = 0;

	while (true)
	{

		//------------RMT GET RADIO PITCH PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch1 = (rmt_item32_t*)xRingbufferReceive(rb_ch1, &rx_size, RMT_RX_WAIT_TICKS);
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
	rmt_get_ringbuf_handler(rmt_rx_ch2.channel, &rb_ch2);
	rmt_rx_start(rmt_rx_ch2.channel, true);
	//--------RMT CONFIG RADIO THR CHANNEL-----------------//

	size_t rx_size = 0;

	while (true)
	{
		//------------RMT GET RADIO THR PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch2 = (rmt_item32_t*)xRingbufferReceive(rb_ch2, &rx_size, RMT_RX_WAIT_TICKS);
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
	rmt_get_ringbuf_handler(rmt_rx_ch3.channel, &rb_ch3);
	rmt_rx_start(rmt_rx_ch3.channel, true);
	//--------RMT CONFIG RADIO YAW CHANNEL-----------------//

	size_t rx_size = 0;

	while (true)
	{

		//------------RMT GET RADIO YAW PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch3 = (rmt_item32_t*)xRingbufferReceive(rb_ch3, &rx_size, RMT_RX_WAIT_TICKS);
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
	rmt_get_ringbuf_handler(rmt_rx_ch4.channel, &rb_ch4);
	rmt_rx_start(rmt_rx_ch4.channel, true);
	//--------RMT CONFIG RADIO 5TH_CHAN CHANNEL-----------------//

	size_t rx_size = 0;

	while (true)
	{

		//------------RMT GET RADIO 5TH CHAN PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch4 = (rmt_item32_t*)xRingbufferReceive(rb_ch4, &rx_size, RMT_RX_WAIT_TICKS);
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
	rmt_get_ringbuf_handler(rmt_rx_ch5.channel, &rb_ch5);
	rmt_rx_start(rmt_rx_ch5.channel, true);
	//--------RMT CONFIG RADIO 6TH_CHAN CHANNEL-----------------//

	size_t rx_size = 0;

	while (true)
	{
		//------------RMT GET RADIO 6TH CHAN PULSE DURATION-----------//
		rx_size = 0;
		rmt_item32_t* item_ch5 = (rmt_item32_t*)xRingbufferReceive(rb_ch5, &rx_size, RMT_RX_WAIT_TICKS);
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
	while (true)
	{
		processRunMotors();
		delay(9);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
	}
	vTaskDelete(NULL);
}

void processTest() {


	digitalWrite(PIN_LED, HIGH);
	delay(500);
	digitalWrite(PIN_LED, LOW);
	delay(500);

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

	// supply your own gyro offsets here, scaled for min sensitivity
	//Jeff Rowberg's IMU_Zero sketch is used to calculate those values
	//mpu.setXGyroOffset(171);
	//mpu.setYGyroOffset(55);
	//mpu.setZGyroOffset(13);
	//mpu.setXAccelOffset(-2068);
	//mpu.setYAccelOffset(-250);
	//mpu.setZAccelOffset(1067);

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
		//Serial.println(fifoCount);
		if (fifoCount >= packetSize)
		{

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


			mpu.dmpGetQuaternion(&q, fifoBuffer);
			mpu.dmpGetGravity(&gravity, &q);
			mpu.dmpGetAccel(&aa, fifoBuffer);
			mpu.dmpGetGyro(gg, fifoBuffer);
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			mpu.dmpGetEuler(euler, &q);


		}


	}
}

void processMapRxDCtoCmd()
{
	if (statusRx == statusType_Normal)
	{
		cmdRxPitch = mapping(dutyCycle_Pitch, DC_PITCH_MIN, DC_PITCH_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);  
		cmdRxRoll = mapping(dutyCycle_Roll, DC_ROLL_MIN, DC_ROLL_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);
		cmdRxYaw = mapping(dutyCycle_Yaw, DC_YAW_MIN, DC_YAW_MAX, CMD_YAW_MIN, CMD_YAW_MAX);
		cmdRxThr = mapping(dutyCycle_Thr, DC_THR_MIN, DC_THR_MAX, CMD_THR_MIN, CMD_THR_MAX);

		cmdRx5thCh = mapping(dutyCycle_Rx5thCh, DC_5TH_CH_MIN, DC_5TH_CH_MAX, -CMD_5TH_CH_MAX, CMD_5TH_CH_MAX);
		cmdRx6thCh = mapping(dutyCycle_Rx6thCh, DC_6TH_CH_MIN, DC_6TH_CH_MAX, -CMD_6TH_CH_MAX, CMD_6TH_CH_MAX);



		//This below code segment is written to have smoother and much feasible commands for quadcopter
#ifdef COMMAND_CALIBRATION
		if (abs(cmdRxRoll) > 1)
		{
			cmdRxPitchRollAngle = atan(abs(cmdRxPitch / cmdRxRoll));
			cmdRxPitchRollSF = cos(min(cmdRxPitchRollAngle, M_PI_2 - cmdRxPitchRollAngle));


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
		modeQuad = modeQuadARMED;
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

	if (modeQuad == modeQuadARMED && cmdMotorThr > CMD_THR_TAKEOFF)
	{
		pidRatePitch.SetFlightMode(true);
		pidRateRoll.SetFlightMode(true);
		pidRateYaw.SetFlightMode(true);
		pidAnglePitch.SetFlightMode(true);
		pidAngleRoll.SetFlightMode(true);
		pidAngleYaw.SetFlightMode(true);
		pidVelAlt.SetFlightMode(true);
		pidAccAlt.SetFlightMode(true);
	}
	else
	{
		pidRatePitch.SetFlightMode(false);
		pidRateRoll.SetFlightMode(false);
		pidRateYaw.SetFlightMode(false);
		pidAnglePitch.SetFlightMode(false);
		pidAngleRoll.SetFlightMode(false);
		pidAngleYaw.SetFlightMode(false);
		pidVelAlt.SetFlightMode(false);
		pidAccAlt.SetFlightMode(false);
	}
}

void processPID()
{
	prePIDprocesses();

	getBodyToEulerAngularRates();

	//Set bypass values to euler angle rates
	pidVars.angleRoll.d_bypass = qc.eulerRate.phi;
	pidVars.anglePitch.d_bypass = qc.eulerRate.theta;
	pidVars.angleYaw.d_bypass = qc.eulerRate.psi;

	//Pitch Roll Yaw Angle PID

	pidVars.angleRoll.setpoint = cmdMotorRoll;
	pidVars.angleRoll.sensedVal = qc.euler.phi * 180 / M_PI;
	pidAngleRoll.Compute();
	pidVars.angleRoll.outputFiltered = basicFilter(pidVars.angleRoll.output, pidVars.angleRoll.outputFilterConstant, pidVars.angleRoll.outputFiltered);


	pidVars.anglePitch.setpoint = cmdMotorPitch;
	pidVars.anglePitch.sensedVal = qc.euler.theta * 180 / M_PI;
	pidAnglePitch.Compute();
	pidVars.anglePitch.outputFiltered = basicFilter(pidVars.anglePitch.output, pidVars.anglePitch.outputFilterConstant, pidVars.anglePitch.outputFiltered);

	pidVars.angleYaw.setpoint = cmdMotorYaw;
	pidVars.angleYaw.sensedVal = qc.euler.psi * 180 / M_PI;
	pidAngleYaw.Compute();
	pidVars.angleYaw.outputFiltered = basicFilter(pidVars.angleYaw.output, pidVars.angleYaw.outputFilterConstant, pidVars.angleYaw.outputFiltered);

	//Transform angle pid outputs to body coordinate axis in order to get correct body angular rate setpoints
	transformAnglePIDoutputsToBody();

	//Pitch Roll Yaw Rate PID

	pidVars.ratePitch.setpoint = rateCmd.y;
	pidVars.ratePitch.sensedVal = qc.gyro.y;
	pidRatePitch.Compute();


	pidVars.rateRoll.setpoint = rateCmd.x;
	pidVars.rateRoll.sensedVal = qc.gyro.x;
	pidRateRoll.Compute();


	pidVars.rateYaw.setpoint = rateCmd.z;
	pidVars.rateYaw.sensedVal = qc.gyro.z;
	pidRateYaw.Compute();


	pidVars.velAlt.d_bypass = qc.accelWorld.z / 8.36;  // cm/second^2
	pidVars.velAlt.setpoint = cmdRx6thCh;
	pidVars.velAlt.sensedVal = qc.velWorld.z;  // cm/second
	pidVelAlt.Compute();
	pidVars.velAlt.outputFiltered = basicFilter(pidVars.velAlt.output, pidVars.velAlt.outputFilterConstant, pidVars.velAlt.outputFiltered);


	//Transform vel pid outputs to body coordinate axis in order to get correct acceleration wrt world
	transformVelPIDoutputsToBody();

	pidVars.accAlt.setpoint = accelCmd.z;   // cmd/second^2
	pidVars.accAlt.sensedVal = qc.accel.z / 8.3;  // cm/second^2
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
	pidVars.ratePitch.f1 = PID_RATE_PITCH_F1_DEFAULT;
	pidVars.ratePitch.f2 = PID_RATE_PITCH_F2_DEFAULT;
	pidVars.ratePitch.outputFilterConstant = PID_RATE_PITCH_OUT_FILT_CONSTANT;
	pidVars.ratePitch.output = 0;
	pidVars.ratePitch.outputCompensated = 0;

	pidVars.anglePitch.Kp = PID_ANGLE_PITCH_KP;
	pidVars.anglePitch.Ki = PID_ANGLE_PITCH_KI;
	pidVars.anglePitch.Kd = PID_ANGLE_PITCH_KD;
	pidVars.anglePitch.outputLimitMin = PID_ANGLE_PITCH_OUTMIN;
	pidVars.anglePitch.outputLimitMax = PID_ANGLE_PITCH_OUTMAX;
	pidVars.anglePitch.f1 = PID_ANGLE_PITCH_F1_DEFAULT;
	pidVars.anglePitch.f2 = PID_ANGLE_PITCH_F2_DEFAULT;
	pidVars.anglePitch.outputFilterConstant = PID_ANGLE_PITCH_OUT_FILT_CONSTANT;
	pidVars.anglePitch.output = 0;
	pidVars.anglePitch.outputCompensated = 0;

	pidVars.rateRoll.Kp = PID_RATE_ROLL_KP;
	pidVars.rateRoll.Ki = PID_RATE_ROLL_KI;
	pidVars.rateRoll.Kd = PID_RATE_ROLL_KD;
	pidVars.rateRoll.outputLimitMin = PID_RATE_ROLL_OUTMIN;
	pidVars.rateRoll.outputLimitMax = PID_RATE_ROLL_OUTMAX;
	pidVars.rateRoll.f1 = PID_RATE_ROLL_F1_DEFAULT;
	pidVars.rateRoll.f2 = PID_RATE_ROLL_F2_DEFAULT;
	pidVars.rateRoll.outputFilterConstant = PID_RATE_ROLL_OUT_FILT_CONSTANT;
	pidVars.rateRoll.output = 0;
	pidVars.rateRoll.outputCompensated = 0;

	pidVars.angleRoll.Kp = PID_ANGLE_ROLL_KP;
	pidVars.angleRoll.Ki = PID_ANGLE_ROLL_KI;
	pidVars.angleRoll.Kd = PID_ANGLE_ROLL_KD;
	pidVars.angleRoll.outputLimitMin = PID_ANGLE_ROLL_OUTMIN;
	pidVars.angleRoll.outputLimitMax = PID_ANGLE_ROLL_OUTMAX;
	pidVars.angleRoll.f1 = PID_ANGLE_ROLL_F1_DEFAULT;
	pidVars.angleRoll.f2 = PID_ANGLE_ROLL_F2_DEFAULT;
	pidVars.angleRoll.outputFilterConstant = PID_ANGLE_ROLL_OUT_FILT_CONSTANT;
	pidVars.angleRoll.output = 0;
	pidVars.angleRoll.outputCompensated = 0;

	pidVars.rateYaw.Kp = PID_RATE_YAW_KP;
	pidVars.rateYaw.Ki = PID_RATE_YAW_KI;
	pidVars.rateYaw.Kd = PID_RATE_YAW_KD;
	pidVars.rateYaw.outputLimitMin = PID_RATE_YAW_OUTMIN;
	pidVars.rateYaw.outputLimitMax = PID_RATE_YAW_OUTMAX;
	pidVars.rateYaw.f1 = PID_RATE_YAW_F1_DEFAULT;
	pidVars.rateYaw.f2 = PID_RATE_YAW_F2_DEFAULT;
	pidVars.rateYaw.outputFilterConstant = PID_RATE_YAW_OUT_FILT_CONSTANT;
	pidVars.rateYaw.output = 0;
	pidVars.rateYaw.outputCompensated = 0;

	pidVars.angleYaw.Kp = PID_ANGLE_YAW_KP;
	pidVars.angleYaw.Ki = PID_ANGLE_YAW_KI;
	pidVars.angleYaw.Kd = PID_ANGLE_YAW_KD;
	pidVars.angleYaw.outputLimitMin = PID_ANGLE_YAW_OUTMIN;
	pidVars.angleYaw.outputLimitMax = PID_ANGLE_YAW_OUTMAX;
	pidVars.angleYaw.f1 = PID_ANGLE_YAW_F1_DEFAULT;
	pidVars.angleYaw.f2 = PID_ANGLE_YAW_F2_DEFAULT;
	pidVars.angleYaw.outputFilterConstant = PID_ANGLE_YAW_OUT_FILT_CONSTANT;
	pidVars.angleYaw.output = 0;
	pidVars.angleYaw.outputCompensated = 0;


	pidVars.velAlt.Kp = PID_VEL_ALT_KP;
	pidVars.velAlt.Ki = PID_VEL_ALT_KI;
	pidVars.velAlt.Kd = PID_VEL_ALT_KD;
	pidVars.velAlt.outputLimitMin = PID_VEL_ALT_OUTMIN;
	pidVars.velAlt.outputLimitMax = PID_VEL_ALT_OUTMAX;
	pidVars.velAlt.f1 = PID_VEL_ALT_F1_DEFAULT;
	pidVars.velAlt.f2 = PID_VEL_ALT_F2_DEFAULT;
	pidVars.velAlt.outputFilterConstant = PID_VEL_ALT_OUT_FILT_CONSTANT;
	pidVars.velAlt.output = 0;
	pidVars.velAlt.outputCompensated = 0;

	pidVars.accAlt.Kp = PID_ACC_ALT_KP;
	pidVars.accAlt.Ki = PID_ACC_ALT_KI;
	pidVars.accAlt.Kd = PID_ACC_ALT_KD;
	pidVars.accAlt.outputLimitMin = CMD_THR_ARM_START;
	pidVars.accAlt.outputLimitMax = CMD_THR_MAX;
	pidVars.accAlt.f1 = PID_ACC_ALT_F1_DEFAULT;
	pidVars.accAlt.f2 = PID_ACC_ALT_F2_DEFAULT;
	pidVars.accAlt.outputFilterConstant = PID_ACC_ALT_OUT_FILT_CONSTANT;
	pidVars.accAlt.output = 0;
	pidVars.accAlt.outputCompensated = 0;
}

void initPIDtuning()
{
	pidRatePitch.SetOutputLimits(pidVars.ratePitch.outputLimitMin, pidVars.ratePitch.outputLimitMax);
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);
	pidRatePitch.SetF1(pidVars.ratePitch.f1);
	pidRatePitch.SetF2(pidVars.ratePitch.f2);

	pidAnglePitch.SetOutputLimits(pidVars.anglePitch.outputLimitMin, pidVars.anglePitch.outputLimitMax);
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);
	pidAnglePitch.SetF1(pidVars.anglePitch.f1);
	pidAnglePitch.SetF2(pidVars.anglePitch.f2);

	pidRateRoll.SetOutputLimits(pidVars.rateRoll.outputLimitMin, pidVars.rateRoll.outputLimitMax);
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);
	pidRateRoll.SetF1(pidVars.rateRoll.f1);
	pidRateRoll.SetF2(pidVars.rateRoll.f2);

	pidAngleRoll.SetOutputLimits(pidVars.angleRoll.outputLimitMin, pidVars.angleRoll.outputLimitMax);
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);
	pidAngleRoll.SetF1(pidVars.angleRoll.f1);
	pidAngleRoll.SetF2(pidVars.angleRoll.f2);

	pidRateYaw.SetOutputLimits(pidVars.rateYaw.outputLimitMin, pidVars.rateYaw.outputLimitMax);
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);
	pidRateYaw.SetF1(pidVars.rateYaw.f1);
	pidRateYaw.SetF2(pidVars.rateYaw.f2);

	pidAngleYaw.SetOutputLimits(pidVars.angleYaw.outputLimitMin, pidVars.angleYaw.outputLimitMax);
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);
	pidAngleYaw.SetF1(pidVars.angleYaw.f1);
	pidAngleYaw.SetF2(pidVars.angleYaw.f2);


	pidVelAlt.SetOutputLimits(pidVars.velAlt.outputLimitMin, pidVars.velAlt.outputLimitMax);
	pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);
	pidVelAlt.SetF1(pidVars.velAlt.f1);
	pidVelAlt.SetF2(pidVars.velAlt.f2);

	pidAccAlt.SetOutputLimits(pidVars.accAlt.outputLimitMin, pidVars.accAlt.outputLimitMax);
	pidAccAlt.SetTunings(pidVars.accAlt.Kp, pidVars.accAlt.Ki, pidVars.accAlt.Kd);
	pidAccAlt.SetF1(pidVars.accAlt.f1);
	pidAccAlt.SetF2(pidVars.accAlt.f2);
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
	else if (autoModeStatus == autoModeAltToOFF)
	{
		if (abs(cmdMotorThr - cmdRxThr) < 8)
		{
			autoModeStatus = autoModeOFF;
		}
		else if (cmdMotorThr > cmdRxThr)
		{
			cmdMotorThr -= 0.3;
		}
		else
		{
			cmdMotorThr += 0.3;
		}
	}
	else
	{
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

float basicFilter(float data, float filterConstant, float filteredData)
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

void transformAnglePIDoutputsToBody()
{
	//rateCmd.x = pidVars.angleRoll.outputFiltered - sin(mpu.euler.theta) * pidVars.angleYaw.outputFiltered;
	//rateCmd.y = cos(mpu.euler.phi) * pidVars.anglePitch.outputFiltered + sin(mpu.euler.phi)*cos(mpu.euler.theta)*pidVars.angleYaw.outputFiltered;
	//rateCmd.z = -sin(mpu.euler.phi) * pidVars.anglePitch.outputFiltered + cos(mpu.euler.phi)*cos(mpu.euler.theta)*pidVars.angleYaw.outputFiltered;

	rateCmd.x = pidVars.angleRoll.outputFiltered;
	rateCmd.y = pidVars.anglePitch.outputFiltered;
	rateCmd.z = pidVars.angleYaw.outputFiltered;

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

	accelCmd.z = (pidVars.velAlt.outputFiltered + 1000);

}

void handleAutoModeCommands()
{
#ifdef MY_RX_TX_IS_6_CHANNEL
	if (modeQuad == modeQuadARMED && (statusRx == statusType_Normal) && (cmdRx5thCh > (CMD_5TH_CH_MAX - 10)))
	{
		autoModeStatus = autoModeAltitude;
	}
	else if (autoModeStatus != autoModeOFF)
	{
		//If previous autoModeStatus is Altitude, then change to transition state
		autoModeStatus = autoModeAltToOFF;
	}
#else	
	if (modeQuad == modeQuadARMED && MsgT01.message.coWorkerTxPacket.statusGS == statusType_Normal)
	{
		autoModeStatus = MsgT01.message.udpT01RelayPacket.autoModeCommand;
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
			pidVars.velAlt.outputCompensated = pidVars.velAlt.output*PID_THR_BATT_SCALE_FACTOR;

			pwmMicroSeconds(M_FL_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_FR_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BR_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BL_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);


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

void calculate_pid_thr_batt_scale_factor() //this function will be modified to include battery voltage compensation also
{
	float total_thr_cmd = cmdMotorThr + pidVars.velAlt.outputCompensated;   // compensated output or normal output, will be discussed later

	if (total_thr_cmd <= CMD_THR_MAX && total_thr_cmd >= CMD_THR_MIN)
		PID_THR_BATT_SCALE_FACTOR = (CMD_THR_MAX - total_thr_cmd) / (CMD_THR_MAX - CMD_THR_MIN) + 0.2;
	else
		PID_THR_BATT_SCALE_FACTOR = 0.4;

	//Use linear interpolation for battery voltage level compensation
	//Make sure that voltage read is in valid range of operation
	if (batteryVoltageInVolts > 9.0 && batteryVoltageInVolts < 13.2)
	{
		PID_THR_BATT_SCALE_FACTOR = PID_THR_BATT_SCALE_FACTOR * 12.0 / batteryVoltageInVolts;
	}
}

void pwmMicroSeconds(int _pwm_channel, int _microseconds)
{
	ledcWrite(_pwm_channel, _microseconds*PWM_MICROSECONDS_TO_BITS);
}