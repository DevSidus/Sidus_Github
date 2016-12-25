#pragma once
#include <string.h>
#include "cMsgUdpT01.h"
#include "structMsgCoWorkerTx.h"
using namespace std;

#pragma pack(push, 1)
struct structMsgT01
{
	char startChar1;
	char startChar2;
	structMsgCoWorkerTx coWorkerTxPacket;
	structMsgUdpT01 udpT01RelayPacket;
	char endChar;
};
#pragma pack(pop)


class cMsgT01
{
public:
	cMsgT01();
	structMsgT01 message;
	unsigned char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgT01();
};
