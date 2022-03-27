#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpTAndroid
{
	unsigned char saveHomePos;
};
#pragma pack(pop)


class cMsgUdpTAndroid
{
public:
	cMsgUdpTAndroid();
	structMsgUdpTAndroid message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgUdpTAndroid();
};