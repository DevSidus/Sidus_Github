#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpRAndroid
{
	unsigned int timeStamp;

	unsigned int modeQuad;
	unsigned int autoModeStatus;
	unsigned int homePointSelected;
	unsigned int posHoldAvailable;

	float batteryVoltage;

	float mpuRoll;
	float mpuPitch;
	float mpuYaw;

	float baroAlt;

	float compassHdg;

	float gpsLat;
	float gpsLon;
	float gpsAlt;
	float gpsVelN;
	float gpsVelE;
	float gpsPosAccuracy;
	float gpsVelAccuracy;

	float quadVelocityWorldX;
	float quadPositionWorldX;
	float quadVelocityWorldY;
	float quadPositionWorldY;
	float quadVelocityWorldZ;
	float quadPositionWorldZ;
	

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
