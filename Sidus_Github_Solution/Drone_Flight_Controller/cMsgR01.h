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
	short pidRatePitchOutput;
	short pidRatePitchPresult;
	short pidRatePitchIresult;
	short pidRatePitchDresult;
	unsigned char pidRatePitchF1;
	unsigned char pidRatePitchF2;

	unsigned char pidAnglePitchKp;
	unsigned char pidAnglePitchKi;
	unsigned char pidAnglePitchKd;
	short pidAnglePitchOutput;
	short pidAnglePitchPresult;
	short pidAnglePitchIresult;
	short pidAnglePitchDresult;
	unsigned char pidAnglePitchF1;
	unsigned char pidAnglePitchF2;
	unsigned char pidAnglePitchOutFilter;

	unsigned char pidRateRollKp;
	unsigned char pidRateRollKi;
	unsigned char pidRateRollKd;
	short pidRateRollOutput;
	short pidRateRollPresult;
	short pidRateRollIresult;
	short pidRateRollDresult;
	unsigned char pidRateRollF1;
	unsigned char pidRateRollF2;

	unsigned char pidAngleRollKp;
	unsigned char pidAngleRollKi;
	unsigned char pidAngleRollKd;
	short pidAngleRollOutput;
	short pidAngleRollPresult;
	short pidAngleRollIresult;
	short pidAngleRollDresult;
	unsigned char pidAngleRollF1;
	unsigned char pidAngleRollF2;
	unsigned char pidAngleRollOutFilter;
	
	unsigned char pidRateYawKp;
	unsigned char pidRateYawKi;
	unsigned char pidRateYawKd;
	short pidRateYawOutput;
	short pidRateYawPresult;
	short pidRateYawIresult;
	short pidRateYawDresult;
	unsigned char pidRateYawF1;
	unsigned char pidRateYawF2;

	unsigned char pidAngleYawKp;
	unsigned char pidAngleYawKi;
	unsigned char pidAngleYawKd;
	short pidAngleYawOutput;
	short pidAngleYawPresult;
	short pidAngleYawIresult;
	short pidAngleYawDresult;
	unsigned char pidAngleYawF1;
	unsigned char pidAngleYawF2;
	unsigned char pidAngleYawOutFilter;

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
