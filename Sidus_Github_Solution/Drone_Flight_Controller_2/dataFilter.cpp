/*
Data Filtering Function
*/

#include "dataFilter.h"

double dataFilter(vector<double> input, const double filterCoefficient[])
{
	double output = 0.0;

	for (unsigned i = 0; i < input.size(); ++i)
		output = output + input[i] * filterCoefficient[i];

	return output;
}

double diffFilter(vector<double> input, const double filterCoefficient[], double deltaTimeSec)
{
	double output = 0.0;

	for (unsigned i = 0; i < input.size(); ++i)
		output = output + input[i] * filterCoefficient[i];

	return output / deltaTimeSec;
}