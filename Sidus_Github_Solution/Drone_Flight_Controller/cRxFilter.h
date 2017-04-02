#pragma once
#include "Arduino.h"

class cRxFilter
{
private:
	int size=7;
	int dataArray[7];
	int counter;
	void sort();

public:
	int process(int);
	cRxFilter();
	~cRxFilter();
};

