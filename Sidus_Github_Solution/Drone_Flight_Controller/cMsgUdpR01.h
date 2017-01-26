#pragma once
#include <string.h>
#include "cMsgR01.h"
#include "cMsgCoWorkerTx.h"
using namespace std;

#pragma pack(push, 1)
struct structMsgUdpR01
{
	structMsgCoWorkerTx coWorkerTxPacket;
	structMsgR01 serialR01RelayPacket;
};
#pragma pack(pop)


class cMsgUdpR01
{
public:
	cMsgUdpR01();
	structMsgUdpR01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgUdpR01();
};
