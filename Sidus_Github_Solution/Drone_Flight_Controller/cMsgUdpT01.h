#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpT01
{
	unsigned char pidCommandState;
	unsigned char pidRatePitchRollKp;
	unsigned char pidRatePitchRollKi;
	unsigned char pidRatePitchRollKd;
	unsigned char pidRatePitchRollF1;
	unsigned char pidRatePitchRollF2;

	unsigned char pidAnglePitchRollKp;
	unsigned char pidAnglePitchRollKi;
	unsigned char pidAnglePitchRollKd;
	unsigned char pidAnglePitchRollF1;
	unsigned char pidAnglePitchRollF2;

	unsigned char pidRateYawKp;
	unsigned char pidRateYawKi;
	unsigned char pidRateYawKd;
	unsigned char pidRateYawF1;
	unsigned char pidRateYawF2;

	unsigned char pidAngleYawKp;
	unsigned char pidAngleYawKi;
	unsigned char pidAngleYawKd;
	unsigned char pidAngleYawF1;
	unsigned char pidAngleYawF2;

	unsigned char pidAnglePitchRollOutFilter;
	unsigned char pidAngleYawOutFilter;

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
