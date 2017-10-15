#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpRAndroid
{
	unsigned int timeStamp;

	unsigned char modeQuad;
	unsigned char autoModeStatus;
	unsigned char posHoldAvailable;

	float batteryVoltage;

	float mpuRoll;
	float mpuPitch;
	float mpuYaw;

	float baroTemp;
	float baroAlt;

	float compassHdg;

	float gpsLat;
	float gpsLon;
	float gpsAlt;
	float homeLat;
	float homeLon;
	float homeAlt;
	float gpsVelN;
	float gpsVelE;
	float gpsPosAccuracy;
	float gpsVelAccuracy;

	float quadVelocityWorldX;   // kalman filtered output
	float quadPositionWorldX;   // kalman filtered output
	float quadVelocityWorldY;   // kalman filtered output
	float quadPositionWorldY;   // kalman filtered output
	float quadVelocityWorldZ;   // kalman filtered output
	float quadPositionWorldZ;   // kalman filtered output
	

};
#pragma pack(pop)


class cMsgUdpRAndroid
{
public:
	cMsgUdpRAndroid();
	structMsgUdpRAndroid message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgUdpRAndroid();
};
