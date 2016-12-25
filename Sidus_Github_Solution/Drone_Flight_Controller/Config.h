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

#define		RX_DATATIME_THESHOLD		1000

#define		CMD_PITCH_MIN	-45.0
#define		CMD_PITCH_MAX	+45.0
#define		DC_PITCH_MIN	1100.0
#define		DC_PITCH_MAX	1900.0

#define		CMD_ROLL_MIN	-45.0
#define		CMD_ROLL_MAX	+45.0
#define		DC_ROLL_MIN		1100.0
#define		DC_ROLL_MAX		1900.0

#define		CMD_THR_MIN		1000.0
#define		CMD_THR_MAX		1900.0
#define		DC_THR_MIN		1100.0
#define		DC_THR_MAX		1900.0

#define		CMD_YAW_MIN		-45.0
#define		CMD_YAW_MAX		+45.0
#define		DC_YAW_MIN		1100.0
#define		DC_YAW_MAX		1900.0




//Enum Type Definitions, two mcu config files may be merged!
typedef enum
{
	statusType_Normal = 0,
	statusType_NotInitiated = 1,
	statusType_InitFail = 2,
	statusType_Fail = 3,
	statusType_UnreliableData = 4
}statusType;

typedef enum
{
	modeQuadSAFE = 0,
	modeQuadDirCmd = 1,
	modeQuadARMED = 2,
	modeQuadCmpClb = 3,

}modeQuadType;


//Global Variable Declarations
int test_task_counter = 0;

uint32_t startTime_Thr, dutyCycle_Thr;
uint32_t startTime_Pitch, dutyCycle_Pitch;
uint32_t startTime_Roll, dutyCycle_Roll;
uint32_t startTime_Yaw, dutyCycle_Yaw;

uint32_t rxLastDataTime;

float cmdPitch = 0, cmdRoll = 0, cmdThr = 0, cmdYaw = 0;

//Status related declarations
unsigned char statusQuad;
unsigned char statusRx;