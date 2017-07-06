#pragma once
#include "Arduino.h"

class cRxFilter
{
private:
	int size = 7;
	int dataArray[7];
	int counter;
	void sort();
	int maxVal;

public:
	int process(int);
	cRxFilter(int);
	~cRxFilter();
};
