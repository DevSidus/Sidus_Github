#pragma once
#include <string.h>
#include "cMsgUdpT01.h"
using namespace std;

#pragma pack(push, 1)
struct structMsgT01
{
	char startChar1;
	char startChar2;

	structMsgUdpT01 udpT01RelayPacket;

	unsigned char mpuStatus;

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
	
	int gpsData;

	/*Not needed for now, to be implemented if required
	//Euler Angles(psi, theta, phi) in Radians
	float mpuEulerPsi;
	float mpuEulerTheta;
	float mpuEulerPhi;
	*/

	char endChar;
};
#pragma pack(pop)


class cMsgT01
{
public:
	cMsgT01();
	structMsgT01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgT01();
};
