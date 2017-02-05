#pragma once
#include "Arduino.h"

class cRxFilter
{
private:
	unsigned short dataArray[3];
	short counter;

public:
	unsigned short process(unsigned short);
	cRxFilter();
	~cRxFilter();
};

