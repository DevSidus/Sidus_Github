#pragma once
#include <string.h>
using namespace std;

#pragma pack(push, 1)
struct structMsgR01
{
	char startChar1;
	char startChar2;
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
