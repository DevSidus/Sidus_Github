#pragma once
#include <string.h>
using namespace std;

class cSerialParse
{
public:
	cSerialParse(short, char, char, char);
	~cSerialParse();
	int size;
	void Push(unsigned char* dataToPush, int _length);
	void Update();
	bool getParsedData(unsigned char *, int);
	char startChar1;
	char startChar2;
	char endChar;
private:
	short pointerIndex;
	unsigned char *dataArray;
	unsigned char *parsedDataArray;
	short packetSize;
	int arraySize;
	bool dataFresh;
};

