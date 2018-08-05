#pragma once
/*
This header file define all the configurable variables including constants, pin mapping, etc.
*/
#include "cDataFilter.h"

//Wifi, UDP Definitions
//#define			WIFI_SSID						"SUPERONLINE_WiFi_4661"
//#define			WIFI_PASS						"JMXPFCTAY3YV"
//#define			DEFAULT_GROUND_STATION_IP		"192.168.1.22"

//If it is desired to make the drone as Access Point, uncomment the following line
#define		DRONE_AP

#define		DRONE_AP_NAME					"SigmaDrone"
#define		DRONE_AP_PASS					"sidus12345"

// Ground Station IP Setting and possible SSIDs
//#define		DEFAULT_GROUND_STATION_IP		"255.255.255.255" // Boradcast for all networks
#define		DEFAULT_GROUND_STATION_IP		"192.168.4.255" // DRONE_AP Broadcast for internal network
//#define		DEFAULT_GROUND_STATION_IP		"192.168.0.255" // YANIKS HOUSE
//#define		DEFAULT_GROUND_STATION_IP		"172.20.10.2" // AAGCA

//#define		WIFI_SSID						"YANIKS HOUSE"
//#define		WIFI_PASS						"YanikTurkiye06"
//#define		WIFI_SSID						"khorfo_net"
//#define		WIFI_PASS						"ahmet_ipek_12082004"
//#define		WIFI_SSID						"AAGCA"
//#define		WIFI_PASS						"ahmet(12082004)"

#define		UDP_PORT						8080

#define		DRONE_WEIGHT					1500.0     //in grams

//Comment out SIDUS_ANDROID_PROTOCOL to establish an Android connection
#define		SIDUS_PROTOCOL
//#define		SIDUS_ANDROID_PROTOCOL

//Comment out MAVLINK_PROTOCOL to establish a Mavlink Connection (SIDUS_PROTOCOL and SIDUS_ANDROID_PROTOCOL should be commented)
//#define		MAVLINK_PROTOCOL

//Comment out below line if you do not want command calibration
#define		COMMAND_CALIBRATION
#define		CMD_MAX_ATTITUDE_IN_RADIANS		M_PI_4		//define max attitude as 45 degrees

//Comment out below line if you have 6 CH RX_TX
#define		MY_RX_TX_IS_6_CHANNEL

//Comment out below line if you want to use SD Card (USensor will not be used)
#define		USE_SD_CARD

//Comment out one of the following barometer hardware selections
//#define		BAROMETER_MS5611
//#define		BAROMETER_BMP180
#define		BAROMETER_BMP280

#ifdef BAROMETER_BMP180
#define		EXISTING_ALTITUDE			950    //in meters from sea-level
double sealevelPress = 1000;
double computedAlt=0;
bool barometer_initial_measurement = true;
#endif // BAROMETER_BMP180
#ifdef BAROMETER_BMP280
#define		SEALEVEL_PRESS				1013.25
#endif

//Comment out one of the following defines which is your FCB and be sure corresponding offsets and constants are inserted in config file

// FCB 3.1 Sensor Settings for TR Boards
//#define SN_031001
//#define SN_031002
//#define SN_031003

// FCB 3.1 Sensor Settings for US Boards
//#define SN_031001US
//#define SN_031002US

// FCB 3.2 Sensor Settings for TR Boards
//#define SN_032001
//#define SN_032002
#define SN_032003

// FCB 3.2 Sensor Settings for US Boards
//#define SN_032001US


#ifdef SN_031001
	#define mpuXAccelOffset				-2553
	#define mpuYAccelOffset				-989
	#define mpuZAccelOffset				1689
	#define mpuXGyroOffset				88
	#define mpuYGyroOffset				-26
	#define mpuZGyroOffset				-5

	double compassHdgXoffset = 92;
	double compassHdgYoffset = 134;
	double compassHdgZoffset = -191;
	double compassHdgXrange = 1000;
	double compassHdgYrange = 1006;
	double compassHdgZrange = 907;
#endif // SN_031001
#ifdef SN_031002
	#define mpuXAccelOffset				-195
	#define mpuYAccelOffset				-669
	#define mpuZAccelOffset				1631
	#define mpuXGyroOffset				59
	#define mpuYGyroOffset				-22
	#define mpuZGyroOffset				-12

	double compassHdgXoffset = 92;
	double compassHdgYoffset = 134;
	double compassHdgZoffset = -191;
	double compassHdgXrange = 1000;
	double compassHdgYrange = 1006;
	double compassHdgZrange = 907;
#endif // SN_031002
#ifdef SN_031003
	#define mpuXAccelOffset				-4580
	#define mpuYAccelOffset				-4589
	#define mpuZAccelOffset				1517
	#define mpuXGyroOffset				74
	#define mpuYGyroOffset				-17
	#define mpuZGyroOffset				-18

	double compassHdgXoffset = 32;
	double compassHdgYoffset = 97;
	double compassHdgZoffset = 249;
	double compassHdgXrange = 1243;
	double compassHdgYrange = 1278;
	double compassHdgZrange = 1117;
#endif // SN_031003
#ifdef SN_031001US
	#define mpuXAccelOffset				1671
	#define mpuYAccelOffset				-287
	#define mpuZAccelOffset				703
	#define mpuXGyroOffset				1
	#define mpuYGyroOffset				-73
	#define mpuZGyroOffset				-14

	double compassHdgXoffset = -47.50;
	double compassHdgYoffset = 196.50;
	double compassHdgZoffset = -124.50;
	double compassHdgXrange = 1065.00;
	double compassHdgYrange = 1017.00;
	double compassHdgZrange = 1007.00;
#endif // SN_031001US
#ifdef SN_031002US
	#define mpuXAccelOffset				1671
	#define mpuYAccelOffset				-287
	#define mpuZAccelOffset				703
	#define mpuXGyroOffset				1
	#define mpuYGyroOffset				-73
	#define mpuZGyroOffset				-14

	double compassHdgXoffset = -47.50;
	double compassHdgYoffset = 196.50;
	double compassHdgZoffset = -124.50;
	double compassHdgXrange = 1065.00;
	double compassHdgYrange = 1017.00;
	double compassHdgZrange = 1007.00;
#endif // SN_031002US
#ifdef SN_032001
	#define mpuXAccelOffset				-4418
	#define mpuYAccelOffset				2279
	#define mpuZAccelOffset				1147
	#define mpuXGyroOffset				-38
	#define mpuYGyroOffset				-22
	#define mpuZGyroOffset				81

	double compassHdgXoffset = 92;
	double compassHdgYoffset = 134;
	double compassHdgZoffset = -191;
	double compassHdgXrange = 1000;
	double compassHdgYrange = 1006;
	double compassHdgZrange = 907;
#endif // SN_032001
#ifdef SN_032001US
	#define mpuXAccelOffset				823
	#define mpuYAccelOffset				-821
	#define mpuZAccelOffset				1769
	#define mpuXGyroOffset				-51
	#define mpuYGyroOffset				70
	#define mpuZGyroOffset				-28

	double compassHdgXoffset = 43.00;
	double compassHdgYoffset = 79.00;
	double compassHdgZoffset = 5.00;
	double compassHdgXrange = 834.00;
	double compassHdgYrange = 672.00;
	double compassHdgZrange = 1022.00;
#endif // SN_032001US

#ifdef SN_032002
	#define mpuXAccelOffset				-2121
	#define mpuYAccelOffset				1091
	#define mpuZAccelOffset				1184
	#define mpuXGyroOffset				38
	#define mpuYGyroOffset				25
	#define mpuZGyroOffset				15

	double compassHdgXoffset = 23;
	double compassHdgYoffset = 72;
	double compassHdgZoffset = -198;
	double compassHdgXrange = 1246;
	double compassHdgYrange = 1225;
	double compassHdgZrange = 1175;
#endif // SN_032002

#ifdef SN_032003
	#define mpuXAccelOffset				-1869
	#define mpuYAccelOffset				-1983
	#define mpuZAccelOffset				1023
	#define mpuXGyroOffset				70
	#define mpuYGyroOffset				-65
	#define mpuZGyroOffset				13

	double compassHdgXoffset = -66;
	double compassHdgYoffset = 103;
	double compassHdgZoffset = 28;
	double compassHdgXrange = 1377;
	double compassHdgYrange = 1239;
	double compassHdgZrange = 1190;
#endif // SN_032003


// Define gravity parameters
#define		MPU_G_MAPPING_IN_BITS				8192.0 // 1 G
#define		GRAVITY_IN_METER_PER_SECOND2		9.80665

#define		WIFI_CONNECTION_THRESHOLD	 10000

// Define Earth Parameters
#define		REA_SEMI_MAJOR_AXIS					6378137.0 // The semi-major axis
#define		FLATTENNING							1 / 298.257223563 // The flattening factor
#define		REB_SEMI_MINOR_AXIS					REA_SEMI_MAJOR_AXIS * (1 - FLATTENNING) // The semi-minor axis
#define		ECCENTRICITY						sqrt(pow(REA_SEMI_MAJOR_AXIS, 2) - pow(REB_SEMI_MINOR_AXIS, 2)) / REA_SEMI_MAJOR_AXIS // The first eccentricity

#define		ACC_BITS_TO_M_SECOND2				GRAVITY_IN_METER_PER_SECOND2 / MPU_G_MAPPING_IN_BITS
//MS5611 Barometer Definitions
#define		BAROMETER_INIT_THRESHOLD	2000
#define		BARO_TEMP_MIN				-20
#define		BARO_TEMP_MAX				100
#define		BARO_ALT_MIN				-100
#define		BARO_ALT_MAX				4000

//HMC5883L Compass Definitions
#define		COMPASS_OFFSET_X_DEFAULT	304
#define		COMPASS_OFFSET_Y_DEFAULT	-360

//GPS UBOX/NMEA Definitions
#define		UBOX

//Pin Definitions
#define		PIN_LED				0
#define		PIN_BATTERY			36

#define		PIN_RX_ROLL			4
#define		PIN_RX_PITCH		39	
#define		PIN_RX_THR			34
#define		PIN_RX_YAW			35
#define		PIN_RX_5TH_CHAN		32
#define		PIN_RX_6TH_CHAN		33

#define		PIN_M_FL			25
#define		PIN_M_FR			26
#define		PIN_M_BR			27
#define		PIN_M_BL			14

#define		PIN_BUZZER			12
#define		PIN_SDCARD_CS		5

#define		PIN_MPU_POWER_ON	13
#define		PIN_MCU_SDA			21
#define		PIN_MCU_SCL			22

#define		PIN_ULTSENS_ECHO	15
#define		PIN_ULTSENS_TRIG	2

#define		PWM_FREQ		100
#define		PWM_DEPTH		16

#define		PWM_MICROSECONDS_TO_BITS	pow(2, PWM_DEPTH)*PWM_FREQ/pow(10,6)

#define		MPU_DATATIME_THRESHOLD		500
#define		GS_CON_LOST_THRESHOLD      500


#define		M_FL_CHANNEL		1
#define		M_FR_CHANNEL		2
#define		M_BR_CHANNEL		3
#define		M_BL_CHANNEL		4
#define		BUZZER_PWM_CHANNEL		5

#define		RX_DATATIME_THRESHOLD		800

#define		CMD_ATTITUDE_MAX 

#define		CMD_RX_PITCH_ROLL_MAX	+45.0
#define		CMD_AUTO_PITCH_ROLL_MAX	+30.0

#define		DC_PITCH_MIN	1100.0
#define		DC_PITCH_MAX	1900.0

#define		DC_ROLL_MIN		1100.0
#define		DC_ROLL_MAX		1900.0

#define		CMD_THR_MIN		1050.0
#define		CMD_THR_MAX		1900.0
#define		DC_THR_MIN		1100.0
#define		DC_THR_MAX		1900.0

#define		ALT_VEL_ZERO_CMD_GAP		100.0
#define		ALT_VEL_ZERO_CMD_MIN     (CMD_THR_MIN + CMD_THR_MAX - ALT_VEL_ZERO_CMD_GAP)/2.0
#define		ALT_VEL_ZERO_CMD_MAX     (CMD_THR_MIN + CMD_THR_MAX + ALT_VEL_ZERO_CMD_GAP)/2.0

#define		CMD_THR_TAKEOFF 1300.0

#define		CMD_YAW_MIN		-45.0
#define		CMD_YAW_MAX		+45.0
#define		DC_YAW_MIN		1100.0
#define		DC_YAW_MAX		1900.0

#define		CMD_5TH_CH_MAX		+100.0
#define		DC_5TH_CH_MIN		1050.0
#define		DC_5TH_CH_MAX		1950.0

#define		CMD_6TH_CH_MAX		+100.0
#define		DC_6TH_CH_MIN		1050.0
#define		DC_6TH_CH_MAX		1950.0


#define		CMD_THR_ARM_START	CMD_THR_MIN+(CMD_THR_MAX-CMD_THR_MIN)/10

#define		CMD_MODE_CHANGE_THR_GAP		50
#define		CMD_MODE_CHANGE_ANGLE_GAP	20

#define		PID_RATE_PITCH_KP			0.65
#define		PID_RATE_PITCH_KI			0.0
#define		PID_RATE_PITCH_KD			0.05
#define		PID_RATE_PITCH_OUTMIN		-250
#define		PID_RATE_PITCH_OUTMAX		250

#define		PID_ANGLE_PITCH_KP			4.0
#define		PID_ANGLE_PITCH_KI			0.0
#define		PID_ANGLE_PITCH_KD			0.0
#define		PID_ANGLE_PITCH_OUTMIN		-250
#define		PID_ANGLE_PITCH_OUTMAX		250

#define		PID_RATE_ROLL_KP			0.65
#define		PID_RATE_ROLL_KI			0.0
#define		PID_RATE_ROLL_KD			0.05
#define		PID_RATE_ROLL_OUTMIN		-250
#define		PID_RATE_ROLL_OUTMAX		250

#define		PID_ANGLE_ROLL_KP			4.0
#define		PID_ANGLE_ROLL_KI			0.0
#define		PID_ANGLE_ROLL_KD			0.0
#define		PID_ANGLE_ROLL_OUTMIN		-250
#define		PID_ANGLE_ROLL_OUTMAX		250


#define		PID_RATE_YAW_KP			1.4
#define		PID_RATE_YAW_KI			0.0
#define		PID_RATE_YAW_KD			0.12
#define		PID_RATE_YAW_OUTMIN		-300
#define		PID_RATE_YAW_OUTMAX		300

#define		PID_ANGLE_YAW_KP			6.0
#define		PID_ANGLE_YAW_KI			0.0
#define		PID_ANGLE_YAW_KD			0.5
#define		PID_ANGLE_YAW_OUTMIN		-150
#define		PID_ANGLE_YAW_OUTMAX		150


#define		PID_POS_ALT_KP			60 // May be tuned better
#define		PID_POS_ALT_KI			0.0
#define		PID_POS_ALT_KD			0.0 // May be tuned better
#define		PID_POS_ALT_OUTMIN		-250
#define		PID_POS_ALT_OUTMAX		250


#define		PID_VEL_ALT_KP			5.0 // May be tuned better
#define		PID_VEL_ALT_KI			0.0
#define		PID_VEL_ALT_KD			0.1 // May be tuned better
#define		PID_VEL_ALT_OUTMIN		-500
#define		PID_VEL_ALT_OUTMAX		500

#define		PID_ACC_ALT_KP			0.22  // Shouldn't be increased than 0.8
#define		PID_ACC_ALT_KI			0.035 // Helpful range is between 0.02 and 0.1
#define		PID_ACC_ALT_KD			0.0044 // It shouldn't be greater than 0.02

#define		PID_ACC_X_KP			1.0 
#define		PID_ACC_X_KI			0.0    
#define		PID_ACC_X_KD			0.05 
#define		PID_ACC_X_OUTMIN		-1000    //need to be revised
#define		PID_ACC_X_OUTMAX		1000     //need to be revised

#define		PID_ACC_Y_KP			1.0  
#define		PID_ACC_Y_KI			0.0    
#define		PID_ACC_Y_KD			0.05 
#define		PID_ACC_Y_OUTMIN		-1000    //need to be revised
#define		PID_ACC_Y_OUTMAX		1000     //need to be revised

#define		PID_VEL_X_KP			5.0 
#define		PID_VEL_X_KI			0.0    
#define		PID_VEL_X_KD			0.0 
#define		PID_VEL_X_OUTMIN		-1000    //need to be revised
#define		PID_VEL_X_OUTMAX		1000     //need to be revised

#define		PID_VEL_Y_KP			5.0  
#define		PID_VEL_Y_KI			0.0    
#define		PID_VEL_Y_KD			0.0
#define		PID_VEL_Y_OUTMIN		-1000    //need to be revised
#define		PID_VEL_Y_OUTMAX		1000     //need to be revised

#define		PID_POS_X_KP			50.0 
#define		PID_POS_X_KI			0.0    
#define		PID_POS_X_KD			0.0 
#define		PID_POS_X_OUTMIN		-500
#define		PID_POS_X_OUTMAX		500

#define		PID_POS_Y_KP			50.0 
#define		PID_POS_Y_KI			0.0    
#define		PID_POS_Y_KD			0.0 
#define		PID_POS_Y_OUTMIN		-500
#define		PID_POS_Y_OUTMAX		500

#define		RX_MAX_PULSE_WIDTH			2075	//in microseconds

#define		MPU_GYRO_DEG_SEC_TO_LSB		16.4     //This value can be used to convert deg/sec to LSB

#define		RESOLUTION_PID_RATE_KP			0.01
#define		RESOLUTION_PID_RATE_KI			0.001
#define		RESOLUTION_PID_RATE_KD			0.001
#define		RESOLUTION_PID_ANGLE_KP			0.1
#define		RESOLUTION_PID_ANGLE_KI			0.01
#define		RESOLUTION_PID_ANGLE_KD			0.001
#define		RESOLUTION_PID_ANGLE_YAW_KD     0.01

#define		RESOLUTION_PID_POS_KP			1
#define		RESOLUTION_PID_POS_KI			1
#define		RESOLUTION_PID_POS_KD			1

#define		RESOLUTION_PID_VEL_KP			0.1
#define		RESOLUTION_PID_VEL_KI			0.01
#define		RESOLUTION_PID_VEL_KD			0.01

#define		RESOLUTION_PID_ACC_KP			0.01
#define		RESOLUTION_PID_ACC_KI			0.001
#define		RESOLUTION_PID_ACC_KD			0.001


#define		SERIAL_COM_SPEED			921600
#define		SERIAL_PARSE_OVF_MULT		3

#define		SERIAL_DEFAULT_GPS_SPEED	9600
#define		SERIAL_GPS_SPEED			115200
#define		GPS_UPDATE_THRESHOLD_TIME	2000

#define     BAT_VOLT_DIV_R1				51.0
#define		BAT_VOLT_DIV_R2				10.0
#define     ADC_ERROR_FACTOR			1.04


#define		RMT_CLK_DIV						80    /*!< RMT counter clock divider */
#define		RMT_FILTER_TICK_THRESHOLD		100    
#define		RMT_IDLE_THRESHOLD				4000  
#define		RMT_RX_BUFFER_SIZE				1000  
#define		RMT_RX_WAIT_TICKS				1000  

#define		BATT_LEVEL_CRITICAL			10.5
#define		BATT_LEVEL_EXIST			2.5

#define     KALMAN_TASK_START_TIME   20000    //in millis

#define		LIDAR_DISTANCE_MIN		1
#define		LIDAR_DISTANCE_MAX		10000
int lidar_distance = 0;    // Distance measured with lidar
bool lidar_available = false;


String		sdcard_filepath;
long lastTime = 0;
long appendPacketCounter = 0;

double		PID_THR_BATT_SCALE_FACTOR = 1.0;
double		PID_BATT_VOLTAGE_SLOPE = -0.1;
double		PID_BATT_MIDDLE_VOLTAGE = 11.5;
float		batteryVoltageInVolts;

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



double barometerTemp = 0.0;
double barometerAlt = 0.0;
double barometerPress;
bool baroReady = false;

double ultrasonicDistance = 0.0;
double ultrasonicDistanceFiltered = 0.0;

double compassHdg;
double compassHdgEstimated;
double compassHdgXmax, compassHdgYmax, compassHdgZmax;
double compassHdgXmin, compassHdgYmin, compassHdgZmin;

short batteryVoltageInBits;
uint32_t udpLastMessageTime;


unsigned long mpuProcessStartTime = 0;
short mpuProcessTaskDuration = 0;
double mpuPitchAngle, mpuRollAngle, mpuYawAngle;
uint32_t mpuLastDataTime, mpuFirstDataTime;
bool mpuFirstDataCheck;

bool wifi_connected = false;
bool udp_connected = false;
int wifi_connection_attempt = 0;

double mpu_gravity_measurement_in_bits = MPU_G_MAPPING_IN_BITS;
bool gravityAcquired = false;


struct structPID
{
	double Kp;
	double Ki;
	double Kd;
	double sensedVal;
	double sensedValDiff;
	double setpoint;
	double setpointDiff;
	double output;
	double outputFiltered;
	double outputCompensated;
	double outputLimitMin;
	double outputLimitMax;
};
struct structSuperPID
{
	structPID ratePitch;
	structPID rateRoll;
	structPID rateYaw;
	structPID anglePitch;
	structPID angleRoll;
	structPID angleYaw;	
	structPID accAlt;
	structPID velAlt;
	structPID posAlt;
	structPID accX;
	structPID accY;
	structPID velX;
	structPID velY;
	structPID posX;
	structPID posY;
}pidVars;

struct structEuler
{
	double psi;
	double theta;
	double phi;
};

struct struct3Daxis
{
	double x;
	double y;
	double z;
};

struct structNEDaxis
{
	double N;
	double E;
	double D;
};

struct structIMU
{
	structEuler euler;
	structEuler eulerRate;
	struct3Daxis gyro;
	struct3Daxis accel;
	struct3Daxis accelWorld;
	structNEDaxis accelNED;
	struct3Daxis accelWorldEstimated;
	structNEDaxis accelNEDEstimated;
	struct3Daxis velWorld; // May be used later
	struct3Daxis velWorldEstimated;
	structNEDaxis velNEDEstimated;
	struct3Daxis posWorld; // May be used later
	struct3Daxis posWorldEstimated;
	structNEDaxis posNEDEstimated;
	struct3Daxis gyroDiff;
	struct3Daxis accelWorldDiff;
}qc;

struct structGPS
{
	double lat;
	double lon;
	double alt; ///< [m], Height above mean sea level
	uint16_t hdop;
	uint16_t vdop;
	uint16_t sog;
	uint16_t cog;
	uint8_t satellites_visible;
	uint8_t gpsFixType;
	bool gpsIsFix;
	struct3Daxis ecefCoordinate; // Earth-Centered Earth-Fixed (ECEF) Cartesian coordinate
	structNEDaxis nedCoordinate; // Local North-East-Down (NED) Cartesian coordinate
	double nE; // The prime vertical radius of curvature
	double  posAccuracy;			///< [m], Horizontal accuracy estimate
	double  altAccuracy;			///< [m], Vertical accuracy estimate
	structNEDaxis nedVelocity;   ///< [m/s], NED north, east and down velocity			
	double  velAccuracy;			///< [m/s], Speed accuracy estimate
}qcGPS;

struct structPOI
{
	double lat;
	double lon;
	double alt; ///< [m], Height above mean sea level
	struct3Daxis ecefCoordinate; // Earth-Centered Earth-Fixed (ECEF) Cartesian coordinate
	double nE; // The prime vertical radius of curvature
};

structPOI homePoint; // Home Point
structPOI destinationPoint; // Destination Point

bool setHomePoint = false;
bool homePointSelected = false;
unsigned int averagingHomePointStartTime; // in ms
const unsigned int averagingHomePointDuration = 1000; // in ms
bool gpsPositionAvailable = false;
bool gpsVelocityAvailable = false;

typedef enum
{
	noFix = 0,
	deadReckoning = 1,
	fix2D = 2,
	fix3D = 3,
	gnssDeadReckoning = 4,
	timeOnlyFix = 5
}gpsFixType;

struct structMAVLINK
{
	uint32_t custom_mode;
	uint8_t type;
	uint8_t autopilot;
	uint8_t base_mode;
	uint8_t system_status;
}qcMavlink;

struct3Daxis angleCmd;
struct3Daxis angleCmdDiff;
struct3Daxis rateCmd;
struct3Daxis rateCmdDiff;
struct3Daxis posCmd;
struct3Daxis posCmdDiff;
struct3Daxis velCmd;
struct3Daxis velCmdDiff;
struct3Daxis accelCmd;
struct3Daxis accelCmdDiff;

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
	modeQuadSpare = 3,
	modeQuadPreARM = 4,

}modeQuadType;

typedef enum
{
	autoModeOFF = 0,
	autoModeAltitude = 1,
}autoModeType;

typedef enum
{
	pidCommandNoAction = 0,
	pidCommandApplyRatePitchRoll = 1,
	pidCommandApplyAnglePitchRoll = 2,
	pidCommandApplyRateYaw = 3,
	pidCommandApplyAngleYaw = 4,
	pidCommandApplyAll = 5,
	pidCommandApplyVelAlt = 6,
	pidCommandApplyAccAlt = 7,
	pidCommandApplyPosAlt = 8,
	pidCommandApplyAccPos = 9,
}pidCommandType;

typedef enum
{
	buzzerMelodySafe = 0,
	buzzerMelodyArmed = 1,
	buzzerMelodyDirCmd = 2,
	buzzerMelodyBatLow = 3,
}buzzerMelodyType;


//Global Variable Declarations
int test_task_counter = 0;

uint32_t now_microsec = 0;

uint32_t startTime_Thr;
uint32_t startTime_Pitch;
uint32_t startTime_Roll;
uint32_t startTime_Yaw;

unsigned short dutyCycle_Thr;
unsigned short dutyCycle_Pitch;
unsigned short dutyCycle_Roll;
unsigned short dutyCycle_Yaw;

#ifdef MY_RX_TX_IS_6_CHANNEL
	uint32_t startTime_Rx5thCh;
	uint32_t startTime_Rx6thCh;
	unsigned short dutyCycle_Rx5thCh;
	unsigned short dutyCycle_Rx6thCh;
#endif

uint32_t rxLastDataTime;

double cmdRxPitch = 0, cmdRxRoll = 0, cmdRxThr = 0, cmdRxYaw = 0;
double cmdRx5thCh = 0, cmdRx6thCh = 0;



double cmdRxPitchCalibrated = 0, cmdRxRollCalibrated = 0;
double cmdRxPitchCalibratedInRad = 0, cmdRxRollCalibratedInRad = 0;
double cmdRxPitchRollAngle=0, cmdRxPitchRollSF=0, cmdRxPitchRollXfactor;

double cmdMotorPitch = 0, cmdMotorRoll = 0, cmdMotorThr = CMD_THR_MIN, cmdMotorYaw = 0;

//Status related declarations
unsigned char modeQuad;
unsigned char autoModeStatus;
unsigned char statusRx;
bool posHoldAvailable = false;
bool velHoldAvailable = false;

//StatusType Definitions
unsigned char statusBaro;
unsigned char statusMpu;
unsigned char statusCompass;
unsigned char statusUdp;
unsigned char statusGS;


typedef enum
{
	filterType_Diff100Hz = 0,
	filterType_Diff200Hz = 1,
	filterType_LPF = 2,
}filterType;

cDataFilter filtObjGyroDiffX(filterType_Diff200Hz);
cDataFilter filtObjGyroDiffY(filterType_Diff200Hz);
cDataFilter filtObjGyroDiffZ(filterType_Diff200Hz);
#define DELTATIME_GYRO_DIFF	 0.005

cDataFilter filtObjRateCmdDiffX(filterType_Diff100Hz);
cDataFilter filtObjRateCmdDiffY(filterType_Diff100Hz);
cDataFilter filtObjRateCmdDiffZ(filterType_Diff100Hz);
#define DELTATIME_RATECMD_DIFF	 0.01

cDataFilter filtObjAngleCmdDiffX(filterType_Diff100Hz);
cDataFilter filtObjAngleCmdDiffY(filterType_Diff100Hz);
cDataFilter filtObjAngleCmdDiffZ(filterType_Diff100Hz); // will be used later
#define DELTATIME_ANGLECMD_DIFF	 0.01

cDataFilter filtObjAccelWorldDiffX(filterType_Diff100Hz);
cDataFilter filtObjAccelWorldDiffY(filterType_Diff100Hz);
cDataFilter filtObjAccelWorldDiffZ(filterType_Diff100Hz);
#define DELTATIME_ACCELWORLD_DIFF	 0.01

cDataFilter filtObjAccelCmdDiffX(filterType_Diff100Hz);
cDataFilter filtObjAccelCmdDiffY(filterType_Diff100Hz);
cDataFilter filtObjAccelCmdDiffZ(filterType_Diff100Hz);
#define DELTATIME_ACCELCMD_DIFF	 0.01

cDataFilter filtObjVelCmdDiffX(filterType_Diff100Hz);
cDataFilter filtObjVelCmdDiffY(filterType_Diff100Hz);
cDataFilter filtObjVelCmdDiffZ(filterType_Diff100Hz);
#define DELTATIME_VELCMD_DIFF	 0.01

cDataFilter filtObjPosCmdDiffX(filterType_Diff100Hz);
cDataFilter filtObjPosCmdDiffY(filterType_Diff100Hz);
cDataFilter filtObjPosCmdDiffZ(filterType_Diff100Hz);
#define DELTATIME_POSCMD_DIFF	 0.01

// These filters are not necessary
//cDataFilter filtObjAnglePIDoutX(filterType_LPF);
//cDataFilter filtObjAnglePIDoutY(filterType_LPF);
//cDataFilter filtObjAnglePIDoutZ(filterType_LPF);

// These filters are not necessary
//cDataFilter filtObjPosPIDoutX(filterType_LPF);
//cDataFilter filtObjPosPIDoutY(filterType_LPF);
//cDataFilter filtObjPosPIDoutZ(filterType_LPF);

// These filters are not necessary
//cDataFilter filtObjVelPIDoutX(filterType_LPF);
//cDataFilter filtObjVelPIDoutY(filterType_LPF);
//cDataFilter filtObjVelPIDoutZ(filterType_LPF);

// These filters are not necessary
//cDataFilter filtObjAccPIDoutX(filterType_LPF);
//cDataFilter filtObjAccPIDoutY(filterType_LPF);
// Acc PID Out Z is not filtered to prevent delay

cDataFilter filtObjUltrasonicDist(filterType_LPF);


// UBlox GPS UBX Commands
// Command sending packet to the receiver to change baudrate to 115200
// CFG-PRT packet
byte ubxPortConfigPacketPart1[28] = {
	0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2,
	0x01, 0x00, 0x07, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0x7E
};
byte ubxPortConfigPacketPart2[9] = {
	0xB5, 0x62, 0x06, 0x00, 0x01, 0x00, 0x01, 0x08, 0x22
};

// Command sending packet to the receiver to change frequency to 10 Hz
// CFG-RATE packet
byte ubxRateConfigPacketPart1[14] = {
	0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12
};
byte ubxRateConfigPacketPart2[8] = {
	0xB5, 0x62, 0x06, 0x08, 0x00, 0x00, 0x0E, 0x30
};

// Command sending packet to the receiver to enable NAV-PVT messages
// CFG-MSG packet
byte ubxMessageConfigPacketPart1[16] = { 
	0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x07, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x18, 0xE1
};
//
byte ubxMessageConfigPacketPart2[10] = { 
	0xB5, 0x62, 0x06, 0x01, 0x02, 0x00, 0x01, 0x07, 0x11, 0x3A
};

