#include "cRxFilter.h"



cRxFilter::cRxFilter()
{
	dataArray[0] = 0;
	dataArray[1] = 0;
	dataArray[2] = 0;

	counter = 0;

}


cRxFilter::~cRxFilter()
{
}

unsigned short cRxFilter::process(unsigned short _newData)
{
	dataArray[counter] = _newData;
	counter = (counter + 1) % 3;

	return max(min(dataArray[0], dataArray[1]), min(max(dataArray[0], dataArray[1]), dataArray[2]));

}
