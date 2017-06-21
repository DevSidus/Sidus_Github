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
#include "cMsgUdpR01.h"
#include "cMsgUdpT01.h"

#include "kalmanFilter.h"
#include "dataFilter.h"

//Global Class Definitions
MPU6050 mpu;
MS5611 barometer;
HMC5883L compass;

//The udp library class
WiFiUDP udp;

cMsgUdpR01 MsgUdpR01;
cMsgUdpT01 MsgUdpT01;

PID pidRatePitch(&pidVars.ratePitch.sensedVal, &pidVars.ratePitch.output, &pidVars.ratePitch.setpoint);
PID pidAnglePitch(&pidVars.anglePitch.sensedVal, &pidVars.anglePitch.output, &pidVars.anglePitch.setpoint, &pidVars.anglePitch.d_bypass);
PID pidRateRoll(&pidVars.rateRoll.sensedVal, &pidVars.rateRoll.output, &pidVars.rateRoll.setpoint);
PID pidAngleRoll(&pidVars.angleRoll.sensedVal, &pidVars.angleRoll.output, &pidVars.angleRoll.setpoint, &pidVars.angleRoll.d_bypass);
PID pidRateYaw(&pidVars.rateYaw.sensedVal, &pidVars.rateYaw.output, &pidVars.rateYaw.setpoint);
PID_YawAngle pidAngleYaw(&pidVars.angleYaw.sensedVal, &pidVars.angleYaw.output, &pidVars.angleYaw.setpoint, &pidVars.angleYaw.d_bypass);

PID pidVelAlt(&pidVars.velAlt.sensedVal, &pidVars.velAlt.output, &pidVars.velAlt.setpoint, &pidVars.velAlt.d_bypass);
PID_AccAlt pidAccAlt(&pidVars.accAlt.sensedVal, &pidVars.accAlt.output, &pidVars.accAlt.setpoint);

cBuzzerMelody buzzer(PIN_BUZZER, BUZZER_PWM_CHANNEL);

SemaphoreHandle_t xI2CSemaphore;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(SERIAL_COM_SPEED);
	Serial.println("Serial started");
	analogRead(4);
	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	//pinMode(PIN_BATTERY, INPUT);
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


	if (xI2CSemaphore == NULL)  // Check to confirm that the Serial Semaphore has not already been created.
	{
		xI2CSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
		if ((xI2CSemaphore) != NULL)
			xSemaphoreGive((xI2CSemaphore));  // Make the Serial Port available for use, by "Giving" the Semaphore.
	}

	TaskHandle_t task_UDPhandle;

	//Processor 0 Tasks
	xTaskCreatePinnedToCore(task_test, "task_test", 1024, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_0, "task_rx_0", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_1, "task_rx_1", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_2, "task_rx_2", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_3, "task_rx_3", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_4, "task_rx_4", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_rx_5, "task_rx_5", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_mapCmd, "task_mapCmd", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_chkMode, "task_chkMode", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_UDP, "task_UDP", 4096, NULL, 10, &task_UDPhandle, 0);
	xTaskCreatePinnedToCore(task_UDPrx, "task_UDPrx", 4096, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_OTA, "task_OTA", 4096, NULL, 20, NULL, 0);
	xTaskCreatePinnedToCore(task_ADC, "task_ADC", 4096, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_melody, "task_melody", 2048, NULL, 1, NULL, 0);
	xTaskCreatePinnedToCore(task_baro, "task_baro", 2048, NULL, 20, NULL, 0);
	xTaskCreatePinnedToCore(task_2Hz, "task_2Hz", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_kalman, "task_kalman", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_filter, "task_filter", 2048, NULL, 10, NULL, 0);
	xTaskCreatePinnedToCore(task_IoT, "task_IoT", 2048, NULL, 10, NULL, 0);

	//Processor 1 Tasks
	xTaskCreatePinnedToCore(task_mpu, "task_mpu", 10000, NULL, 20, NULL, 1);
	xTaskCreatePinnedToCore(task_compass, "task_compass", 2048, NULL, 10, NULL, 1);
	xTaskCreatePinnedToCore(task_PID, "task_PID", 8192, NULL, 20, NULL, 1);
	xTaskCreatePinnedToCore(task_Motor, "task_Motor", 2048, NULL, 20, NULL, 1);



	//esp_event_loop_init((system_event_cb_t*)&task_UDPhandle, NULL);
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
	statusMpu = statusType_NotInitiated;
	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)4000) == pdTRUE)
	{
		initMPU();
		xSemaphoreGive(xI2CSemaphore);
	}
	while (true)
	{
		//mpuProcessStartTime = micros();
		if (xSemaphoreTake(xI2CSemaphore, (TickType_t)2) == pdTRUE)
		{
			processMpu();
			xSemaphoreGive(xI2CSemaphore);
		}
		//mpuProcessTaskDuration = micros() - mpuProcessStartTime;
		//Serial.println(millis());
		delay(3);
	}
	vTaskDelete(NULL);
	return;
}

void task_baro(void * parameter)
{
	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)4000) == pdTRUE)
	{
		initBarometer();
		xSemaphoreGive(xI2CSemaphore);
	}
	while (true)
	{
		if (xSemaphoreTake(xI2CSemaphore, (TickType_t)2) == pdTRUE)
		{
			processBarometer();
			xSemaphoreGive(xI2CSemaphore);
		}

		delay(15);
	}
	vTaskDelete(NULL);
	return;
}

void task_compass(void * parameter)
{
	if (xSemaphoreTake(xI2CSemaphore, (TickType_t)4000) == pdTRUE)
	{
		initCompass();
		xSemaphoreGive(xI2CSemaphore);
	}

	while (true)
	{
		if (xSemaphoreTake(xI2CSemaphore, (TickType_t)0) == pdTRUE)
		{
			processCompass();
			xSemaphoreGive(xI2CSemaphore);
		}
		delay(35);
	}
	vTaskDelete(NULL);
	return;
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
	rmt_get_ringbuf_handler(rmt_rx_ch0.channel, &rb_ch0);
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
	rmt_get_ringbuf_handler(rmt_rx_ch1.channel, &rb_ch1);
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
	rmt_get_ringbuf_handler(rmt_rx_ch2.channel, &rb_ch2);
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
	rmt_get_ringbuf_handler(rmt_rx_ch3.channel, &rb_ch3);
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
	rmt_get_ringbuf_handler(rmt_rx_ch4.channel, &rb_ch4);
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
	rmt_get_ringbuf_handler(rmt_rx_ch5.channel, &rb_ch5);
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

	delay(2000);
	initUDP();
		while (true)
	{
		if (wifi_connected)
		{
			prepareUDPmessages();
			udp.beginPacket(DEFAULT_GROUND_STATION_IP, UDP_PORT);
			udp.write(MsgUdpR01.dataBytes, sizeof(MsgUdpR01.dataBytes));
			udp.endPacket();
		}
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(10);
	}
	vTaskDelete(NULL);
}

void task_UDPrx(void * parameter)
{
	while (true)
	{
		if (wifi_connected)
		{


			if (udp.parsePacket() >= sizeof(MsgUdpT01.dataBytes))
			{
				udp.read(MsgUdpT01.dataBytes, sizeof(MsgUdpT01.dataBytes));
				MsgUdpT01.setPacket();

				updateUdpMsgVars();

				//Serial.println("udp message received");
				//udpLastMessageTime = millis();
			}
			//processUDPrx();
		}
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(150);
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
		batteryVoltageInBits=analogRead(PIN_BATTERY);
		batteryVoltageInVolts = float(batteryVoltageInBits) * ((BAT_VOLT_DIV_R1 + BAT_VOLT_DIV_R2) / BAT_VOLT_DIV_R2) * 3.3 / 4095 * ADC_ERROR_FACTOR;
		//adcStart(PIN_BATTERY);
		//Serial.println(uxTaskGetStackHighWaterMark(NULL));
		delay(100);
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

void task_2Hz(void * parameter)
{
	statusMpu = statusType_NotInitiated;
	mpuLastDataTime = millis();
	while (true)
	{
		checkMpuGSHealth();
		delay(500);
	}
	vTaskDelete(NULL);
}

void task_kalman(void * parameter)
{
	double F_n[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	double H_n[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

	double Q_n[9] = { 0.0000018, 0, 0, 0, 0.0000018, 0, 0, 0, 0.0000018 };
	double R_n[9] = { 0.000324, 0, 0, 0, 0.000324, 0, 0, 0, 0.000324 };


	double P_n1[9] = { 0.018, 0, 0, 0, 0.018, 0, 0, 0, 0.018 };
	double m_n1[3] = { 0, 0, 0 };

	double m_n[3] = { 0, 0, 0 };
	double P_n[9] = { 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	double y_n[3] = { 0, 0, 0 };
	
	//unsigned long strtTime;
	
	while (true)
	{
		//strtTime = micros();
		y_n[0] = qc.gyro.x;
		y_n[1] = qc.gyro.y;
		y_n[2] = qc.gyro.z;

		kalmanFilter(m_n1, P_n1, y_n, F_n, Q_n, H_n, R_n, m_n, P_n);
		
		qc.gyroKalman.x = m_n[0];
		qc.gyroKalman.y = m_n[1];
		qc.gyroKalman.z = m_n[2];

		memcpy(m_n1, m_n, sizeof(m_n));
		memcpy(P_n1, P_n, sizeof(P_n));

		//Serial.println(micros() - strtTime);

		delay(9);
	}
	vTaskDelete(NULL);
}

void task_filter(void * parameter)
{
	vector<double> gyroXBlock;
	vector<double> gyroYBlock;
	vector<double> gyroZBlock;
	for (int i = 0; i < diffFilterLength; i++) gyroXBlock.push_back(0.0);
	for (int i = 0; i < diffFilterLength; i++) gyroYBlock.push_back(0.0);
	for (int i = 0; i < diffFilterLength; i++) gyroZBlock.push_back(0.0);
	
	unsigned long lastTime = millis();

	while (true)
	{
		unsigned long now = millis();
		unsigned long dTime = (now - lastTime);
		double dTimeInSec = dTime / 1000.0;

		double newGyroXdata = qc.gyro.x;
		gyroXBlock.erase(gyroXBlock.begin());
		gyroXBlock.push_back(newGyroXdata);

		double newGyroYdata = qc.gyro.y;
		gyroYBlock.erase(gyroYBlock.begin());
		gyroYBlock.push_back(newGyroYdata);

		double newGyroZdata = qc.gyro.z;
		gyroZBlock.erase(gyroZBlock.begin());
		gyroZBlock.push_back(newGyroZdata);

		qc.gyroFiltered.x = dataFilter(gyroXBlock, diffFilterCoefficient);
		qc.gyroFiltered.y = dataFilter(gyroYBlock, diffFilterCoefficient);
		qc.gyroFiltered.z = dataFilter(gyroZBlock, diffFilterCoefficient);

		// When Diff Filter is being used
		qc.gyroFiltered.x = qc.gyroFiltered.x / dTimeInSec;
		qc.gyroFiltered.y = qc.gyroFiltered.y / dTimeInSec;
		qc.gyroFiltered.z = qc.gyroFiltered.z / dTimeInSec;

		lastTime = now;

		delay(10);
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

	// supply your own gyro offsets here, scaled for min sensitivity
	//Jeff Rowberg's IMU_Zero sketch is used to calculate those values
	mpu.setXAccelOffset(-2553);
	mpu.setYAccelOffset(-989);
	mpu.setZAccelOffset(1689);
	mpu.setXGyroOffset(88);
	mpu.setYGyroOffset(-26);
	mpu.setZGyroOffset(-5);

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
			mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
			mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
			mpu.dmpGetEuler(euler, &q);

			qc.gyro.x = gg[0];
			qc.gyro.y = -gg[1];
			qc.gyro.z = -gg[2];

			qc.accel.x = aa.x;
			qc.accel.y = aa.y;
			qc.accel.z = aa.z;

			qc.accelBody.x = aaReal.x;
			qc.accelBody.y = aaReal.y;
			qc.accelBody.z = aaReal.z;

			qc.accelWorld.x = aaWorld.x;
			qc.accelWorld.y = aaWorld.y;
			qc.accelWorld.z = aaWorld.z;

			qc.euler.psi = -euler[0];  
			qc.euler.theta = -euler[1]; 
			qc.euler.phi = euler[2];   

		}


	}
}

void processMapRxDCtoCmd()
{
	//Serial.println(cmdRxPitchCalibrated);
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
			Serial.println("Baro init failed");
			return false;
		}
		delay(200);
	}
	statusBaro = statusType_Normal;

	Serial.println("Baro init success");

	delay(100);
	return true;
}

void processBarometer()
{
	//Barometer Data Processing
	barometer.runProcess();
	barometerTemp = barometer.readTemperature(false);
	barometerPress = barometer.readPressure(false);
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
	Vector compassNorm = compass.readNormalize();
	// To calculate heading in degrees. 0 degree indicates North
	compassHdg = atan2(compassNorm.YAxis, compassNorm.XAxis);
}

void initUDP()
{
	Serial.println("Connecting to WiFi network: " + String(WIFI_SSID));

	WiFi.mode(WIFI_STA);
	// delete old config
	WiFi.disconnect(true);
	//register event handler
	WiFi.onEvent(WiFiEvent);

	//Initiate connection
	WiFi.begin(WIFI_SSID, WIFI_PASS);
	delay(200);
	Serial.println("Waiting for WIFI connection...");
}

void WiFiEvent(WiFiEvent_t event) {
	switch (event) {
	case SYSTEM_EVENT_STA_GOT_IP:
		//When connected set 
		Serial.print("WiFi connected! IP address: ");
		Serial.println(WiFi.localIP());
		//initializes the UDP state
		//This initializes the transfer buffer
		udp.begin(WiFi.localIP(), UDP_PORT);
		wifi_connected = true;
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		Serial.println("WiFi lost connection");
		wifi_connected = false;
		if (wifi_connection_attempt < WIFI_CONNECTION_ATTEMPT_LIMIT)
		{
			initUDP();
		}

		wifi_connection_attempt++;
		break;
	}
}

void initOTA()
{
	while (!wifi_connected)
	{
		delay(100);
		if (wifi_connection_attempt >= WIFI_CONNECTION_ATTEMPT_LIMIT)
		{
			//Serial.println("Unable to connect wifi, Suspending OTA Task!");
			//vTaskSuspend(NULL);
		}
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
	Serial.println(WiFi.localIP());
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
		applyPidCommandVelAlt();
		applyPidCommandAccAlt();
		break;

	case pidCommandApplyVelAlt:
		applyPidCommandVelAlt();
		break;

	case pidCommandApplyAccAlt:
		applyPidCommandAccAlt();
		break;
	default:break;
	}
}

void applyPidCommandRatePitchRoll()
{
	//Set Pitch Rate PID parameters
	pidVars.ratePitch.Kp = MsgUdpT01.message.pidRatePitchRollKp * RESOLUTION_PID_RATE_KP;
	pidVars.ratePitch.Ki = MsgUdpT01.message.pidRatePitchRollKi * RESOLUTION_PID_RATE_KI;
	pidVars.ratePitch.Kd = MsgUdpT01.message.pidRatePitchRollKd * RESOLUTION_PID_RATE_KD;
	pidVars.ratePitch.f1 = MsgUdpT01.message.pidRatePitchRollF1 * RESOLUTION_PID_F;
	pidVars.ratePitch.f2 = MsgUdpT01.message.pidRatePitchRollF2 * RESOLUTION_PID_F;
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);
	pidRatePitch.SetF1(pidVars.ratePitch.f1);
	pidRatePitch.SetF2(pidVars.ratePitch.f2);
	//Set Roll Rate PID parameters
	pidVars.rateRoll.Kp = pidVars.ratePitch.Kp;
	pidVars.rateRoll.Ki = pidVars.ratePitch.Ki;
	pidVars.rateRoll.Kd = pidVars.ratePitch.Kd;
	pidVars.rateRoll.f1 = pidVars.ratePitch.f1;
	pidVars.rateRoll.f2 = pidVars.ratePitch.f2;
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);
	pidRateRoll.SetF1(pidVars.rateRoll.f1);
	pidRateRoll.SetF2(pidVars.rateRoll.f2);
}

void applyPidCommandAnglePitchRoll()
{
	//Set Pitch Angle PID parameters
	pidVars.anglePitch.Kp = MsgUdpT01.message.pidAnglePitchRollKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.anglePitch.Ki = MsgUdpT01.message.pidAnglePitchRollKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.anglePitch.Kd = MsgUdpT01.message.pidAnglePitchRollKd * RESOLUTION_PID_ANGLE_KD;
	pidVars.anglePitch.f1 = MsgUdpT01.message.pidAnglePitchRollF1 * RESOLUTION_PID_F;
	pidVars.anglePitch.f2 = MsgUdpT01.message.pidAnglePitchRollF2 * RESOLUTION_PID_F;
	pidVars.anglePitch.outputFilterConstant = MsgUdpT01.message.pidAnglePitchRollOutFilter * RESOLUTION_PID_F;
	pidVars.angleRoll.outputFilterConstant = pidVars.anglePitch.outputFilterConstant;
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);
	pidAnglePitch.SetF1(pidVars.anglePitch.f1);
	pidAnglePitch.SetF2(pidVars.anglePitch.f2);
	//Set Roll Angle PID parameters
	pidVars.angleRoll.Kp = pidVars.anglePitch.Kp;
	pidVars.angleRoll.Ki = pidVars.anglePitch.Ki;
	pidVars.angleRoll.Kd = pidVars.anglePitch.Kd;
	pidVars.angleRoll.f1 = pidVars.anglePitch.f1;
	pidVars.angleRoll.f2 = pidVars.anglePitch.f2;
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);
	pidAngleRoll.SetF1(pidVars.angleRoll.f1);
	pidAngleRoll.SetF2(pidVars.angleRoll.f2);
}

void applyPidCommandRateYaw()
{
	//Set Yaw Rate PID parameters
	pidVars.rateYaw.Kp = MsgUdpT01.message.pidRateYawKp * RESOLUTION_PID_RATE_KP;
	pidVars.rateYaw.Ki = MsgUdpT01.message.pidRateYawKi * RESOLUTION_PID_RATE_KI;
	pidVars.rateYaw.Kd = MsgUdpT01.message.pidRateYawKd * RESOLUTION_PID_RATE_KD;
	pidVars.rateYaw.f1 = MsgUdpT01.message.pidRateYawF1 * RESOLUTION_PID_F;
	pidVars.rateYaw.f2 = MsgUdpT01.message.pidRateYawF2 * RESOLUTION_PID_F;
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);
	pidRateYaw.SetF1(pidVars.rateYaw.f1);
	pidRateYaw.SetF2(pidVars.rateYaw.f2);
}

void applyPidCommandAngleYaw()
{
	//Set Yaw Angle PID parameters
	pidVars.angleYaw.Kp = MsgUdpT01.message.pidAngleYawKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.angleYaw.Ki = MsgUdpT01.message.pidAngleYawKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.angleYaw.Kd = MsgUdpT01.message.pidAngleYawKd * RESOLUTION_PID_ANGLE_YAW_KD;
	pidVars.angleYaw.f1 = MsgUdpT01.message.pidAngleYawF1 * RESOLUTION_PID_F;
	pidVars.angleYaw.f2 = MsgUdpT01.message.pidAngleYawF2 * RESOLUTION_PID_F;
	pidVars.angleYaw.outputFilterConstant = MsgUdpT01.message.pidAngleYawOutFilter * RESOLUTION_PID_F;
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);
	pidAngleYaw.SetF1(pidVars.angleYaw.f1);
	pidAngleYaw.SetF2(pidVars.angleYaw.f2);
}

void applyPidCommandVelAlt()
{
	//Set Altitude Velocity PID parameters
	pidVars.velAlt.Kp = MsgUdpT01.message.pidVelAltKp * RESOLUTION_PID_VEL_KP;
	pidVars.velAlt.Ki = MsgUdpT01.message.pidVelAltKi * RESOLUTION_PID_VEL_KI;
	pidVars.velAlt.Kd = MsgUdpT01.message.pidVelAltKd * RESOLUTION_PID_VEL_KD;
	pidVars.velAlt.f1 = MsgUdpT01.message.pidVelAltF1 * RESOLUTION_PID_F;
	pidVars.velAlt.f2 = MsgUdpT01.message.pidVelAltF2 * RESOLUTION_PID_F;
	pidVars.velAlt.outputFilterConstant = MsgUdpT01.message.pidVelAltOutFilter * RESOLUTION_PID_F;
	pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);
	pidVelAlt.SetF1(pidVars.velAlt.f1);
	pidVelAlt.SetF2(pidVars.velAlt.f2);
}

void applyPidCommandAccAlt()
{
	//Set Altitude Position PID parameters
	pidVars.accAlt.Kp = MsgUdpT01.message.pidAccAltKp * RESOLUTION_PID_POS_KP;
	pidVars.accAlt.Ki = MsgUdpT01.message.pidAccAltKi * RESOLUTION_PID_POS_KI;
	pidVars.accAlt.Kd = MsgUdpT01.message.pidAccAltKd * RESOLUTION_PID_POS_KD;
	pidVars.accAlt.f1 = MsgUdpT01.message.pidAccAltF1 * RESOLUTION_PID_F;
	pidVars.accAlt.f2 = MsgUdpT01.message.pidAccAltF2 * RESOLUTION_PID_F;
	pidAccAlt.SetTunings(pidVars.accAlt.Kp, pidVars.accAlt.Ki, pidVars.accAlt.Kd);
	pidAccAlt.SetF1(pidVars.accAlt.f1);
	pidAccAlt.SetF2(pidVars.accAlt.f2);
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
	MsgUdpR01.message.mpuGyroXkalman		= qc.gyroKalman.x;
	MsgUdpR01.message.mpuGyroYkalman		= qc.gyroKalman.y;
	MsgUdpR01.message.mpuGyroZkalman		= qc.gyroKalman.z;
	MsgUdpR01.message.mpuGyroXfilter		= qc.gyroFiltered.x;
	MsgUdpR01.message.mpuGyroYfilter		= qc.gyroFiltered.y;
	MsgUdpR01.message.mpuGyroZfilter		= qc.gyroFiltered.z;
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
	MsgUdpR01.message.compassHdg			= compassHdg;
	MsgUdpR01.message.batteryVoltage    	= batteryVoltageInVolts;
	MsgUdpR01.message.quadVelocityWorldZ	= 0;
	MsgUdpR01.message.quadPositionWorldZ	= 0;
	MsgUdpR01.message.modeQuad				= modeQuad;
	MsgUdpR01.message.autoModeStatus		= autoModeStatus;
	MsgUdpR01.message.statusRx				= statusRx;
	MsgUdpR01.message.rxThrottle			= cmdRxThr;
	MsgUdpR01.message.rxPitch				= cmdRxPitchCalibrated;
	MsgUdpR01.message.rxRoll				= cmdRxRollCalibrated;
	MsgUdpR01.message.rxYaw					= cmdRxYaw;
	MsgUdpR01.message.pidRatePitchKp		= pidVars.ratePitch.Kp / RESOLUTION_PID_RATE_KP;
	MsgUdpR01.message.pidRatePitchKi		= pidVars.ratePitch.Ki / RESOLUTION_PID_RATE_KI;
	MsgUdpR01.message.pidRatePitchKd		= pidVars.ratePitch.Kd / RESOLUTION_PID_RATE_KD;
	MsgUdpR01.message.pidRatePitchOutput	= pidVars.ratePitch.output;
	MsgUdpR01.message.pidRatePitchPresult	= pidRatePitch.Get_P_Result();
	MsgUdpR01.message.pidRatePitchIresult	= pidRatePitch.Get_I_Result();
	MsgUdpR01.message.pidRatePitchDresult	= pidRatePitch.Get_D_Result();
	MsgUdpR01.message.pidRatePitchF1		= pidVars.ratePitch.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidRatePitchF2		= pidVars.ratePitch.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAnglePitchKp		= pidVars.anglePitch.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgUdpR01.message.pidAnglePitchKi		= pidVars.anglePitch.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgUdpR01.message.pidAnglePitchKd		= pidVars.anglePitch.Kd / RESOLUTION_PID_ANGLE_KD;
	MsgUdpR01.message.pidAnglePitchOutput	= pidVars.anglePitch.output;
	MsgUdpR01.message.pidAnglePitchPresult	= pidAnglePitch.Get_P_Result();
	MsgUdpR01.message.pidAnglePitchIresult	= pidAnglePitch.Get_I_Result();
	MsgUdpR01.message.pidAnglePitchDresult	= pidAnglePitch.Get_D_Result();
	MsgUdpR01.message.pidAnglePitchF1		= pidVars.anglePitch.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAnglePitchF2		= pidVars.anglePitch.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAnglePitchOutFilter = pidVars.anglePitch.outputFilterConstant / RESOLUTION_PID_F;
	MsgUdpR01.message.pidRateRollKp			= pidVars.rateRoll.Kp / RESOLUTION_PID_RATE_KP;
	MsgUdpR01.message.pidRateRollKi			= pidVars.rateRoll.Ki / RESOLUTION_PID_RATE_KI;
	MsgUdpR01.message.pidRateRollKd			= pidVars.rateRoll.Kd / RESOLUTION_PID_RATE_KD;
	MsgUdpR01.message.pidRateRollOutput		= pidVars.rateRoll.output;
	MsgUdpR01.message.pidRateRollPresult	= pidRateRoll.Get_P_Result();
	MsgUdpR01.message.pidRateRollIresult	= pidRateRoll.Get_I_Result();
	MsgUdpR01.message.pidRateRollDresult	= pidRateRoll.Get_D_Result();
	MsgUdpR01.message.pidRateRollF1			= pidVars.rateRoll.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidRateRollF2			= pidVars.rateRoll.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleRollKp		= pidVars.angleRoll.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgUdpR01.message.pidAngleRollKi		= pidVars.angleRoll.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgUdpR01.message.pidAngleRollKd		= pidVars.angleRoll.Kd / RESOLUTION_PID_ANGLE_KD;
	MsgUdpR01.message.pidAngleRollOutput	= pidVars.angleRoll.output;
	MsgUdpR01.message.pidAngleRollPresult	= pidAngleRoll.Get_P_Result();
	MsgUdpR01.message.pidAngleRollIresult	= pidAngleRoll.Get_I_Result();
	MsgUdpR01.message.pidAngleRollDresult	= pidAngleRoll.Get_D_Result();
	MsgUdpR01.message.pidAngleRollF1		= pidVars.angleRoll.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleRollF2		= pidVars.angleRoll.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleRollOutFilter = pidVars.angleRoll.outputFilterConstant / RESOLUTION_PID_F;
	MsgUdpR01.message.pidRateYawKp			= pidVars.rateYaw.Kp / RESOLUTION_PID_RATE_KP;
	MsgUdpR01.message.pidRateYawKi			= pidVars.rateYaw.Ki / RESOLUTION_PID_RATE_KI;
	MsgUdpR01.message.pidRateYawKd			= pidVars.rateYaw.Kd / RESOLUTION_PID_RATE_KD;
	MsgUdpR01.message.pidRateYawOutput		= pidVars.rateYaw.output;
	MsgUdpR01.message.pidRateYawPresult		= pidRateYaw.Get_P_Result();
	MsgUdpR01.message.pidRateYawIresult		= pidRateYaw.Get_I_Result();
	MsgUdpR01.message.pidRateYawDresult		= pidRateYaw.Get_D_Result();
	MsgUdpR01.message.pidRateYawF1			= pidVars.rateYaw.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidRateYawF2			= pidVars.rateYaw.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleYawKp			= pidVars.angleYaw.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgUdpR01.message.pidAngleYawKi			= pidVars.angleYaw.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgUdpR01.message.pidAngleYawKd			= pidVars.angleYaw.Kd / RESOLUTION_PID_ANGLE_YAW_KD;
	MsgUdpR01.message.pidAngleYawOutput		= pidVars.angleYaw.output;
	MsgUdpR01.message.pidAngleYawPresult	= pidAngleYaw.Get_P_Result();
	MsgUdpR01.message.pidAngleYawIresult	= pidAngleYaw.Get_I_Result();
	MsgUdpR01.message.pidAngleYawDresult	= pidAngleYaw.Get_D_Result();
	MsgUdpR01.message.pidAngleYawF1			= pidVars.angleYaw.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleYawF2			= pidVars.angleYaw.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAngleYawOutFilter	= pidVars.angleYaw.outputFilterConstant / RESOLUTION_PID_F;
	MsgUdpR01.message.commandedYawAngle		= cmdMotorYaw;
	MsgUdpR01.message.pidVelAltKp			= pidVars.velAlt.Kp / RESOLUTION_PID_VEL_KP;
	MsgUdpR01.message.pidVelAltKi			= pidVars.velAlt.Ki / RESOLUTION_PID_VEL_KI;
	MsgUdpR01.message.pidVelAltKd			= pidVars.velAlt.Kd / RESOLUTION_PID_VEL_KD;
	MsgUdpR01.message.pidVelAltOutput		= pidVars.velAlt.output;
	MsgUdpR01.message.pidVelAltPresult		= pidVelAlt.Get_P_Result();
	MsgUdpR01.message.pidVelAltIresult		= pidVelAlt.Get_I_Result();
	MsgUdpR01.message.pidVelAltDresult		= pidVelAlt.Get_D_Result();
	MsgUdpR01.message.pidVelAltF1			= pidVars.velAlt.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidVelAltF2			= pidVars.velAlt.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAccAltKp			= pidVars.accAlt.Kp / RESOLUTION_PID_POS_KP;
	MsgUdpR01.message.pidAccAltKi			= pidVars.accAlt.Ki / RESOLUTION_PID_POS_KI;
	MsgUdpR01.message.pidAccAltKd			= pidVars.accAlt.Kd / RESOLUTION_PID_POS_KD;
	MsgUdpR01.message.pidAccAltOutput		= pidVars.accAlt.output;
	MsgUdpR01.message.pidAccAltPresult		= pidAccAlt.Get_P_Result();
	MsgUdpR01.message.pidAccAltIresult		= pidAccAlt.Get_I_Result();
	MsgUdpR01.message.pidAccAltDresult		= pidAccAlt.Get_D_Result();
	MsgUdpR01.message.pidAccAltF1			= pidVars.accAlt.f1 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAccAltF2			= pidVars.accAlt.f2 / RESOLUTION_PID_F;
	MsgUdpR01.message.pidAccAltOutFilter	= pidVars.accAlt.outputFilterConstant / RESOLUTION_PID_F;


	MsgUdpR01.getPacket();
}

void checkMpuGSHealth()
{
	if ((millis() - mpuLastDataTime> MPU_DATATIME_THRESHOLD) && statusMpu != statusType_NotInitiated)
	{
		statusMpu = statusType_Fail;
		//try to restart MPU and check mpu status there again
		if (xSemaphoreTake(xI2CSemaphore, (TickType_t)4000) == pdTRUE)
		{
			initMPU();
			xSemaphoreGive(xI2CSemaphore);
		}
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
