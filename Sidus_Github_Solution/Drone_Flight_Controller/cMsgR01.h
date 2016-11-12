#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgR01
{
};
#pragma pack(pop)


class cMsgR01
{
public:
	cMsgR01();
	structMsgR01 message;
	char dataBytes[sizeof(message)];
	void getPacket();
	void setPacket();
	~cMsgR01();
};
