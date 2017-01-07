#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgR01
{
	char startChar1;
	char startChar2;

	unsigned char modeQuad;
	unsigned char statusRx;
	short rxThrottle;
	short rxPitch;
	short rxRoll;
	short rxYaw;
	unsigned char pidRatePitchKp;
	unsigned char pidRatePitchKi;
	unsigned char pidRatePitchKd;
	unsigned char pidRateRollKp;
	unsigned char pidRateRollKi;
	unsigned char pidRateRollKd;
	unsigned char pidRateYawKp;
	unsigned char pidRateYawKi;
	unsigned char pidRateYawKd;
	unsigned char pidAnglePitchKp;
	unsigned char pidAnglePitchKi;
	unsigned char pidAnglePitchKd;
	unsigned char pidAngleRollKp;
	unsigned char pidAngleRollKi;
	unsigned char pidAngleRollKd;
	unsigned char pidAngleYawKp;
	unsigned char pidAngleYawKi;
	unsigned char pidAngleYawKd;
	char endChar;
};
#pragma pack(pop)


class cMsgR01
{
public:
	cMsgR01();
	structMsgR01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgR01();
};
