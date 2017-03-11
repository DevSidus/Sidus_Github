#pragma once
/*
This header file define all the configurable variables including constants, pin mapping, etc.
*/

//Wifi, UDP Definitions
//#define		WIFI_SSID	"khorfo_net"
//#define		WIFI_PASS	"ahmet_ipek_12082004"
#define		WIFI_SSID	"AndroidAP"
#define		WIFI_PASS	"Yavuzzz."
//#define		WIFI_SSID	"kulucka"
//#define		WIFI_PASS	"girisim*"
#define		UDP_PORT	8080
#define		DEFAULT_GROUND_STATION_IP	"192.168.43.84"
//#define		DEFAULT_GROUND_STATION_IP	"192.168.1.9"
//#define		DEFAULT_GROUND_STATION_IP	"172.20.10.2"

//If your IMU is placed inverse, you should uncomment the below definition
//#define		INVERSE_IMU

//MS5611 Barometer Definitions
#define		BAROMETER_INIT_THRESHOLD	2000
#define		BARO_TEMP_MIN				-20
#define		BARO_TEMP_MAX				100
#define		BARO_ALT_MIN				-100
#define		BARO_ALT_MAX				4000

//HMC5883L Compass Definitions
#define		COMPASS_OFFSET_X_DEFAULT	304
#define		COMPASS_OFFSET_Y_DEFAULT	-360

//Pin Definitions
#define		PIN_LED				4
#define		PIN_MPU_POWER_ON	16
#define		PIN_MCU_SDA			12
#define		PIN_MCU_SCL			14

#define		MPU_DATATIME_THESHOLD		1000



//Global Variable Declarations
int test_task_counter = 0;


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
						// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
int16_t gg[3];         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector

float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


unsigned long mpuProcessStartTime = 0;
short mpuProcessTaskDuration = 0;
double mpuPitchAngle, mpuRollAngle, mpuYawAngle;
uint32_t mpuLastDataTime, mpuFirstDataTime;
bool mpuFirstDataCheck;

float barometerTemp, barometerAlt;
int32_t barometerPress;

float compassHdg;

short batteryVoltageInBits;


//Enum Type Definitions, two mcu config files may be merged!
typedef enum
{
	statusType_Normal = 0,
	statusType_NotInitiated = 1,
	statusType_InitFail = 2,
	statusType_Fail = 3,
	statusType_UnreliableData = 4
}statusType;

//StatusType Definitions
unsigned char statusBaro;
unsigned char statusMpu;
unsigned char statusCompass;
unsigned char statusUdp;
