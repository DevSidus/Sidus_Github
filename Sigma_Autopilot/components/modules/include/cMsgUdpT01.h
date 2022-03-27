#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpT01
{
	unsigned char autoModeCommand;

	unsigned char pidCommandState;
	unsigned char pidRatePitchRollKp;
	unsigned char pidRatePitchRollKi;
	unsigned char pidRatePitchRollKd;

	unsigned char pidAnglePitchRollKp;
	unsigned char pidAnglePitchRollKi;
	unsigned char pidAnglePitchRollKd;

	unsigned char pidRateYawKp;
	unsigned char pidRateYawKi;
	unsigned char pidRateYawKd;

	unsigned char pidAngleYawKp;
	unsigned char pidAngleYawKi;
	unsigned char pidAngleYawKd;

	unsigned char pidPosAltKp;
	unsigned char pidPosAltKi;
	unsigned char pidPosAltKd;

	unsigned char pidVelAltKp;
	unsigned char pidVelAltKi;
	unsigned char pidVelAltKd;

	unsigned char pidAccAltKp;
	unsigned char pidAccAltKi;
	unsigned char pidAccAltKd;

	unsigned char pidAccPosKp;
	unsigned char pidAccPosKi;
	unsigned char pidAccPosKd;

	unsigned char saveHomePos;


};
#pragma pack(pop)


class cMsgUdpT01
{
public:
	cMsgUdpT01();
	structMsgUdpT01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgUdpT01();
};
