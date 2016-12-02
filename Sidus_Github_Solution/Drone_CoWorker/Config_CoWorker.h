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
#define PIN_MPU_POWER_ON	15



//Constant Variables
#define SERIAL_PACKET_SIZE	103
#define SERIAL_START_CHAR_1	'$'
#define SERIAL_START_CHAR_2	'<'
#define SERIAL_END_CHAR		'>'


//Global Variable Declarations
int test_task_counter = 0;

