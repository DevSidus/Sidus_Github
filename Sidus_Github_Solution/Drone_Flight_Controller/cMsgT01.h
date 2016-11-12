#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgT01
{
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
