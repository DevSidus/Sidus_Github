#include "cSerialParse.h"

cSerialParse::cSerialParse(short _packetSize, char _startChar1, char _startChar2, char _endChar)
{	
	packetSize = _packetSize;
	startChar1 = _startChar1;
	startChar2 = _startChar2;
	endChar = _endChar;
	arraySize = packetSize * 3;
	pointerIndex = arraySize;
	dataFresh = false;
	dataArray = new unsigned char[arraySize];
	parsedDataArray=new unsigned char[packetSize];
}


cSerialParse::~cSerialParse()
{
}

void cSerialParse::Push(unsigned char* dataToPush, int _length)
{
	for (int i = 0; i < arraySize; i++)
	{
		if (i + _length < arraySize)
		{
			dataArray[i] = dataArray[i + _length];
		}
		else
		{
			dataArray[i] = dataToPush[i + _length - arraySize];
		}
	}
	pointerIndex -= _length;

	Update();
}

void cSerialParse::Update()
{
	if (pointerIndex < 0)
	{
		pointerIndex = 0;
	}

	for (int i = pointerIndex; i <= arraySize - packetSize; i++)
	{

		if (dataArray[i] == startChar1 && dataArray[i + 1] == startChar2 && dataArray[i + packetSize - 1] == endChar)
		{
			memcpy(parsedDataArray, &dataArray[i], packetSize);
			i += (packetSize - 1);
			pointerIndex += packetSize;
			dataFresh = true;
		}
		else
		{
			pointerIndex++;
		}

	}
	
}

bool cSerialParse::getParsedData(unsigned char *data, int size)
{
	bool returnVal = dataFresh;
	memcpy(data, parsedDataArray, size);
	dataFresh = false;
	return returnVal;
}