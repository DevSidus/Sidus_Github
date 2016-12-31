#pragma once
/*
This header file define all the configurable variables including constants, pin mapping, etc.
*/

//Wifi, UDP Definitions
#define WIFI_SSID	"AAGCA"
#define WIFI_PASS	"ahmet(12082004)"
#define UDP_PORT	8080




//Pin Definitions
#define PIN_LED				5

#define		PIN_THR			36	
#define		PIN_PITCH		37	
#define		PIN_ROLL		38
#define		PIN_YAW			39

//uSonic Pin Definitions
#define TRIGGER_PIN			12		//sensor trigger pin
#define ECHO_PIN			13		//sensor echo pin
#define MAX_DISTANCE		200		//MAX distance for USensor


//Constant Variables
#define SERIAL_PACKET_SIZE	103
#define SERIAL_START_CHAR_1	'$'
#define SERIAL_START_CHAR_2	'<'
#define SERIAL_END_CHAR		'>'


//Global Variable Declarations
int test_task_counter = 0;

uint32_t startTime_Thr, dutyCycle_Thr;
uint32_t startTime_Pitch, dutyCycle_Pitch;
uint32_t startTime_Roll, dutyCycle_Roll;
uint32_t startTime_Yaw, dutyCycle_Yaw;

uint32_t rxLastDataTime;