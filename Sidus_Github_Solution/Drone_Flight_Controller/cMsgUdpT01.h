#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpT01
{
	char testVerisi;
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
