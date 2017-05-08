#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgCoWorkerTx
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
	short mpuAccWorldX;
	short mpuAccWorldY;
	short mpuAccWorldZ;

	//Yaw Pitch Roll Measurements in Radians
	float mpuYaw;
	float mpuPitch;
	float mpuRoll;

	float baroTemp;
	float baroAlt;

	float compassHdg;

	short batteryVoltageInBits;

	float quadVelocityWorldZ;   //kalman filtered output
	float quadPositionWorldZ;   //kalman filtered output

};
#pragma pack(pop)