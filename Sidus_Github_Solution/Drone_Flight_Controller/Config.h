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

#define		PIN_RX_THR			36	
#define		PIN_RX_PITCH		37	
#define		PIN_RX_ROLL			38
#define		PIN_RX_YAW			39
#define		PIN_RX_5TH_CHAN		32
#define		PIN_RX_6TH_CHAN		33

#define		PIN_M1			34
#define		PIN_M2			35
#define		PIN_M3			25
#define		PIN_M4			26

#define		PWM_FREQ		50
#define		PWM_DEPTH		16

#define		PWM_MICROSECONDS_TO_BITS	3.277 //pow(2, PWM_DEPTH)*PWM_FREQ/pow(10,6);

#define		M1_CHANNEL		1
#define		M2_CHANNEL		2
#define		M3_CHANNEL		3
#define		M4_CHANNEL		4

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