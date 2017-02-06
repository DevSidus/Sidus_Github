#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpT01
{
	unsigned char pidCommandState;
	unsigned char pidRatePitchKp;
	unsigned char pidRatePitchKi;
	unsigned char pidRatePitchKd;

	unsigned char pidRatePitchF1;
	unsigned char pidRatePitchF2;


	//unsigned char pidRateRollKp;
	//unsigned char pidRateRollKi;
	//unsigned char pidRateRollKd;
	//unsigned char pidRateYawKp;
	//unsigned char pidRateYawKi;
	//unsigned char pidRateYawKd;
	//unsigned char pidAnglePitchKp;
	//unsigned char pidAnglePitchKi;
	//unsigned char pidAnglePitchKd;
	//unsigned char pidAngleRollKp;
	//unsigned char pidAngleRollKi;
	//unsigned char pidAngleRollKd;
	//unsigned char pidAngleYawKp;
	//unsigned char pidAngleYawKi;
	//unsigned char pidAngleYawKd;
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
