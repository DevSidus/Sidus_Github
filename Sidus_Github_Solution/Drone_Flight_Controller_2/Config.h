#pragma once
/*
This header file define all the configurable variables including constants, pin mapping, etc.
*/

//Wifi, UDP Definitions
#define			WIFI_SSID						"khorfo_net"
#define			WIFI_PASS						"ahmet_ipek_12082004"
#define			DEFAULT_GROUND_STATION_IP		"192.168.1.8"
//#define		WIFI_SSID						"AAGCA"
//#define		WIFI_PASS						"ahmet(12082004)"
//#define		DEFAULT_GROUND_STATION_IP		"172.20.10.2"

#define		UDP_PORT						8080

#define		WIFI_CONNECTION_THRESHOLD	 10000

//Comment out below line if you do not want command calibration
#define		COMMAND_CALIBRATION
#define		CMD_MAX_ATTITUDE_IN_RADIANS		M_PI_4		//define max attitude as 45 degrees

//Comment out below line if you have 6 CH RX_TX
#define		MY_RX_TX_IS_6_CHANNEL


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
#define		PIN_LED				5
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

#define		PIN_MPU_POWER_ON	13
#define		PIN_MCU_SDA			21
#define		PIN_MCU_SCL			22

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

#define		DC_PITCH_MIN	1100.0
#define		DC_PITCH_MAX	1900.0

#define		DC_ROLL_MIN		1100.0
#define		DC_ROLL_MAX		1900.0

#define		CMD_THR_MIN		1050.0
#define		CMD_THR_MAX		1900.0
#define		DC_THR_MIN		1100.0
#define		DC_THR_MAX		1900.0

#define		CMD_THR_TAKEOFF 1380.0

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


#define		PID_RATE_PITCH_KP			1.6
#define		PID_RATE_PITCH_KI			0.0
#define		PID_RATE_PITCH_KD			0.13
#define		PID_RATE_PITCH_OUTMIN		-250
#define		PID_RATE_PITCH_OUTMAX		250
#define		PID_RATE_PITCH_F1_DEFAULT	0.0
#define		PID_RATE_PITCH_F2_DEFAULT	0.0
#define		PID_RATE_PITCH_OUT_FILT_CONSTANT	0.0

#define		PID_ANGLE_PITCH_KP			4.0
#define		PID_ANGLE_PITCH_KI			0.0
#define		PID_ANGLE_PITCH_KD			0.0
#define		PID_ANGLE_PITCH_OUTMIN		-250
#define		PID_ANGLE_PITCH_OUTMAX		250
#define		PID_ANGLE_PITCH_F1_DEFAULT	0.0
#define		PID_ANGLE_PITCH_F2_DEFAULT	0.0
#define		PID_ANGLE_PITCH_OUT_FILT_CONSTANT	0.0

#define		PID_RATE_ROLL_KP			1.6
#define		PID_RATE_ROLL_KI			0.0
#define		PID_RATE_ROLL_KD			0.13
#define		PID_RATE_ROLL_OUTMIN		-250
#define		PID_RATE_ROLL_OUTMAX		250
#define		PID_RATE_ROLL_F1_DEFAULT	0.0
#define		PID_RATE_ROLL_F2_DEFAULT	0.0
#define		PID_RATE_ROLL_OUT_FILT_CONSTANT		0.0

#define		PID_ANGLE_ROLL_KP			4.0
#define		PID_ANGLE_ROLL_KI			0.0
#define		PID_ANGLE_ROLL_KD			0.0
#define		PID_ANGLE_ROLL_OUTMIN		-250
#define		PID_ANGLE_ROLL_OUTMAX		250
#define		PID_ANGLE_ROLL_F1_DEFAULT	0.0
#define		PID_ANGLE_ROLL_F2_DEFAULT	0.0
#define		PID_ANGLE_ROLL_OUT_FILT_CONSTANT	0.0


#define		PID_RATE_YAW_KP			1.4
#define		PID_RATE_YAW_KI			0.0
#define		PID_RATE_YAW_KD			0.12
#define		PID_RATE_YAW_OUTMIN		-300
#define		PID_RATE_YAW_OUTMAX		300
#define		PID_RATE_YAW_F1_DEFAULT	0.0
#define		PID_RATE_YAW_F2_DEFAULT	0.3
#define		PID_RATE_YAW_OUT_FILT_CONSTANT		0

#define		PID_ANGLE_YAW_KP			3.0
#define		PID_ANGLE_YAW_KI			0.0
#define		PID_ANGLE_YAW_KD			0.255
#define		PID_ANGLE_YAW_OUTMIN		-150
#define		PID_ANGLE_YAW_OUTMAX		150
#define		PID_ANGLE_YAW_F1_DEFAULT	0.0
#define		PID_ANGLE_YAW_F2_DEFAULT	0.9
#define		PID_ANGLE_YAW_OUT_FILT_CONSTANT	0.3


#define		PID_VEL_ALT_KP			2.0
#define		PID_VEL_ALT_KI			0.0
#define		PID_VEL_ALT_KD			0.0
#define		PID_VEL_ALT_OUTMIN		-500
#define		PID_VEL_ALT_OUTMAX		500
#define		PID_VEL_ALT_F1_DEFAULT	0.0
#define		PID_VEL_ALT_F2_DEFAULT	0.0
#define		PID_VEL_ALT_OUT_FILT_CONSTANT		0

#define		PID_ACC_ALT_KP			0.2
#define		PID_ACC_ALT_KI			0.0
#define		PID_ACC_ALT_KD			0.0
#define		PID_ACC_ALT_F1_DEFAULT	0.0
#define		PID_ACC_ALT_F2_DEFAULT	0.94
#define		PID_ACC_ALT_OUT_FILT_CONSTANT	0.3



#define		RX_MAX_PULSE_WIDTH			2075	//in microseconds

#define		MPU_GYRO_DEG_SEC_TO_LSB		16.4     //This value can be used to convert deg/sec to LSB

#define		RESOLUTION_PID_RATE_KP			0.01
#define		RESOLUTION_PID_RATE_KI			0.001
#define		RESOLUTION_PID_RATE_KD			0.001
#define		RESOLUTION_PID_ANGLE_KP			0.1
#define		RESOLUTION_PID_ANGLE_KI			0.01
#define		RESOLUTION_PID_ANGLE_KD			0.001
#define		RESOLUTION_PID_ANGLE_YAW_KD     0.01
#define		RESOLUTION_PID_F				0.01

#define		RESOLUTION_PID_VEL_KP			0.1
#define		RESOLUTION_PID_VEL_KI			0.01
#define		RESOLUTION_PID_VEL_KD			0.01
#define		RESOLUTION_PID_POS_KP			0.01
#define		RESOLUTION_PID_POS_KI			0.001
#define		RESOLUTION_PID_POS_KD			0.001


#define		SERIAL_COM_SPEED			921600
#define		SERIAL_PARSE_OVF_MULT		3

#define     BAT_VOLT_DIV_R1				51.0
#define		BAT_VOLT_DIV_R2				10.0
#define     ADC_ERROR_FACTOR			1.11


#define		RMT_CLK_DIV						80    /*!< RMT counter clock divider */
#define		RMT_FILTER_TICK_THRESHOLD		100    
#define		RMT_IDLE_THRESHOLD				4000  
#define		RMT_RX_BUFFER_SIZE				1000  
#define		RMT_RX_WAIT_TICKS				1000  

#define		BATT_LEVEL_CRITICAL			10.5
#define		BATT_LEVEL_EXIST			2.5

double		PID_THR_BATT_SCALE_FACTOR = 1.0;
double		PID_BATT_VOLTAGE_SLOPE = -0.15;
double		PID_BATT_MIDDLE_VOLTAGE = 11.5;
float		batteryVoltageInVolts;
double		commandedAltitude = 0;

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



float barometerTemp = 0.0;
double barometerAlt = 0.0;
int32_t barometerPress;

float compassHdg;

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
	float  outputFilterConstant;
	double outputCompensated;
	double outputLimitMin;
	double outputLimitMax;
	float f1;
	float f2;
};
struct structSuperPID
{
	structPID ratePitch;
	structPID rateRoll;
	structPID rateYaw;
	structPID velAlt;
	structPID anglePitch;
	structPID angleRoll;
	structPID angleYaw;	
	structPID accAlt;
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

struct structIMU
{
	structEuler euler;
	structEuler eulerRate;
	struct3Daxis gyro;
	struct3Daxis accel;
	struct3Daxis accelWorld;
	struct3Daxis accelWorldEstimated;
	struct3Daxis accelBody;
	struct3Daxis velWorldEstimated;
	struct3Daxis posWorldEstimated;
	struct3Daxis gyroDiff;
}qc;

struct3Daxis rateCmd;
struct3Daxis rateCmdDiff;
struct3Daxis accelCmd;

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

}modeQuadType;

typedef enum
{
	autoModeOFF = 0,
	autoModeAltitude = 1,
	autoModeAltToOFF = 2,
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

//StatusType Definitions
unsigned char statusBaro;
unsigned char statusMpu;
unsigned char statusCompass;
unsigned char statusUdp;
unsigned char statusGS;

struct struct3Dvector
{
	vector<double> xVector;
	vector<double> yVector;
	vector<double> zVector;
};

struct3Dvector gyroDiffBuffer;
struct3Dvector anglePIDoutputLowpassBuffer;
struct3Dvector rateCmdDiffBuffer;

double deltaTimeGyroDiff; // will be used at Exact Filtering
double deltaTimeRateCmdDiff; // will be used at Exact Filtering