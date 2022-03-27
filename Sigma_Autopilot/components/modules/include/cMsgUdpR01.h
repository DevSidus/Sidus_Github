#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpR01
{
	unsigned int timeStamp;

	unsigned char statusMpu;
	unsigned char statusBaro;
	unsigned char statusCompass;
	unsigned char statusUdp;
	unsigned char statusGS;

	//Gyro Measurements
	short mpuGyroX;
	short mpuGyroY;
	short mpuGyroZ;

	//Accelerometer Measurements with Gravity
	short mpuAccX;
	short mpuAccY;
	short mpuAccZ;

	//Accelerometer Measurements without Gravity
	float mpuAccWorldX;
	float mpuAccWorldY;
	float mpuAccWorldZ;

	//Yaw Pitch Roll Measurements in Radians
	float mpuYaw;
	float mpuPitch;
	float mpuRoll;

	float baroTemp;
	float baroAlt;

	float compassHdg;

	float batteryVoltage;

	float ultrasonicDist;

	// Position Parameter Estimation with Kalman Filter
	float quadAccelerationWorldX; // kalman filtered output
	float quadVelocityWorldX;   // kalman filtered output
	float quadPositionWorldX;   // kalman filtered output
	float quadAccelerationWorldY; // kalman filtered output
	float quadVelocityWorldY;   // kalman filtered output
	float quadPositionWorldY;   // kalman filtered output

	// Altitude Parameter Estimation with Kalman Filter
	float quadAccelerationWorldZ; // kalman filtered output
	float quadVelocityWorldZ;   // kalman filtered output
	float quadPositionWorldZ;   // kalman filtered output
	
	unsigned char modeQuad;
	unsigned char autoModeStatus;
	unsigned char statusRx;
	short rxThrottle;
	short rxPitch;
	short rxRoll;
	short rxYaw;
	short rx6thCh;

	unsigned char pidRatePitchKp;
	unsigned char pidRatePitchKi;
	unsigned char pidRatePitchKd;
	float pidRatePitchOutput;
	float pidRatePitchPresult;
	float pidRatePitchIresult;
	float pidRatePitchDresult;

	unsigned char pidAnglePitchKp;
	unsigned char pidAnglePitchKi;
	unsigned char pidAnglePitchKd;
	float pidAnglePitchOutput;
	float pidAnglePitchPresult;
	float pidAnglePitchIresult;
	float pidAnglePitchDresult;

	unsigned char pidRateYawKp;
	unsigned char pidRateYawKi;
	unsigned char pidRateYawKd;
	float pidRateYawOutput;
	float pidRateYawPresult;
	float pidRateYawIresult;
	float pidRateYawDresult;

	unsigned char pidAngleYawKp;
	unsigned char pidAngleYawKi;
	unsigned char pidAngleYawKd;
	float pidAngleYawOutput;
	float pidAngleYawPresult;
	float pidAngleYawIresult;
	float pidAngleYawDresult;
	float commandedYawAngle;

	unsigned char pidPosAltKp;
	unsigned char pidPosAltKi;
	unsigned char pidPosAltKd;
	float pidPosAltOutput;
	float pidPosAltPresult;
	float pidPosAltIresult;
	float pidPosAltDresult;

	unsigned char pidVelAltKp;
	unsigned char pidVelAltKi;
	unsigned char pidVelAltKd;
	float pidVelAltOutput;
	float pidVelAltPresult;
	float pidVelAltIresult;
	float pidVelAltDresult;

	unsigned char pidAccAltKp;
	unsigned char pidAccAltKi;
	unsigned char pidAccAltKd;
	float pidAccAltOutput;
	float pidAccAltPresult;
	float pidAccAltIresult;
	float pidAccAltDresult;

	unsigned char pidPosXKp;
	unsigned char pidPosXKi;
	unsigned char pidPosXKd;
	float pidPosXOutput;
	float pidPosXPresult;
	float pidPosXIresult;
	float pidPosXDresult;

	unsigned char pidVelXKp;
	unsigned char pidVelXKi;
	unsigned char pidVelXKd;
	float pidVelXOutput;
	float pidVelXPresult;
	float pidVelXIresult;
	float pidVelXDresult;

	unsigned char pidAccXKp;
	unsigned char pidAccXKi;
	unsigned char pidAccXKd;
	float pidAccXOutput;
	float pidAccXPresult;
	float pidAccXIresult;
	float pidAccXDresult;

	//GPS DATA
	unsigned char gpsStatus;
	double gpsLat;
	double gpsLon;
	double gpsAlt;
	double homeLat;
	double homeLon;
	double homeAlt;
	double gpsVelN;
	double gpsVelE;
	double gpsVelD;
	double gpsPosAccuracy;
	double gpsVelAccuracy;
	
	unsigned char posHoldAvailable;
	unsigned char velHoldAvailable;

	//Lidar Distance
	unsigned short lidar_distance;
};
#pragma pack(pop)


class cMsgUdpR01
{
public:
	cMsgUdpR01();
	structMsgUdpR01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgUdpR01();
};

























































































































































