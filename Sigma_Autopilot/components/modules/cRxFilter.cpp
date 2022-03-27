#include "cRxFilter.h"



cRxFilter::cRxFilter(int _maxVal)
{
	dataArray[0] = 0;
	dataArray[1] = 0;
	dataArray[2] = 0;
	dataArray[3] = 0;
	dataArray[4] = 0;
	dataArray[5] = 0;
	dataArray[6] = 0;
	size = 7;
	counter = 0;
	maxVal = _maxVal;

}


cRxFilter::~cRxFilter()
{
}

int cRxFilter::process(int _newData)
{
	//ignore new coming data if it is larger than defined max_pulse_width
	if (_newData < maxVal)
	{
		dataArray[counter] = _newData;
		counter = (counter + 1) % size;

		sort();

		//return _newData;
	}
	return dataArray[((size - 1) / 2)];
}

void cRxFilter::sort() {
	for (int i = 0; i<(size - 1); i++) {
		for (int o = 0; o<(size - (i + 1)); o++) {
			if (dataArray[o] > dataArray[o + 1]) {
				int t = dataArray[o];
				dataArray[o] = dataArray[o + 1];
				dataArray[o + 1] = t;
			}
		}
	}
}