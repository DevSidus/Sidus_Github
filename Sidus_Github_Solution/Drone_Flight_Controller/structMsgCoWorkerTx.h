#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgCoWorkerTx
{
	unsigned char statusMpu;
	unsigned char statusBaro;
	unsigned char statusCompass;
	unsigned char statusUdp;

	//Gyro Measurements
	short mpuGyroX;
	short mpuGyroY;
	short mpuGyroZ;

	//Accelerometer Measurements with Gravity
	short mpuAccX;
	short mpuAccY;
	short mpuAccZ;

	//Accelerometer Measurements without Gravity
	short mpuAccRealX;
	short mpuAccRealY;
	short mpuAccRealZ;

	//Yaw Pitch Roll Measurements in Radians
	float mpuYaw;
	float mpuPitch;
	float mpuRoll;

	float baroTemp;
	float baroAlt;

	float compassHdg;

	short batteryVoltageInBits;

	/*Not needed for now, to be implemented if required
	//Euler Angles(psi, theta, phi) in Radians
	float mpuEulerPsi;
	float mpuEulerTheta;
	float mpuEulerPhi;
	*/
};
#pragma pack(pop)