#include "cDataFilter.h"



cDataFilter::cDataFilter(int _type)
{
	index = 0;
	type = _type;

	if (type == 0) //DiffFilter100Hz
	{
		coeff[0] = -0.104799329538250;
		coeff[1] = -0.0702839469518364;
		coeff[2] = -0.0352678034184209;
		coeff[3] = 0;
		coeff[4] = 0.0352678034184209;
		coeff[5] = 0.0702839469518364;
		coeff[6] = 0.104799329538250;
	}
	else if (type == 1) //DiffFilter200Hz
	{
		coeff[0] = -0.101988983017612;
		coeff[1] = -0.0680787959196394;
		coeff[2] = -0.0340652650142217;
		coeff[3] = 0;
		coeff[4] = 0.0340652650142217;
		coeff[5] = 0.0680787959196394;
		coeff[6] = 0.101988983017612;
	}
	else  //lpf
	{
		coeff[0] = 0.0236315810934875;
		coeff[1] = 0.0927938270028177;
		coeff[2] = 0.232319803922457;
		coeff[3] = 0.302509575962475;
		coeff[4] = 0.232319803922457;
		coeff[5] = 0.0927938270028177;
		coeff[6] = 0.0236315810934875;
	}

	deltaTime = 0.01;   // set default deltaTime 10 ms

	for (int i = 0; i < 7; i++)
	{
		data[i] = 0;
	}
}


cDataFilter::~cDataFilter()
{
}


double cDataFilter::filter(double _val, double _dTime)
{
	double output = 0.0;

	data[index] = _val;
	index = (index + 1) % 7;
	deltaTime = _dTime;

	for (int i = 0; i < 7; ++i)
		output = output + data[(index + i) % 7] * coeff[i];

	if (type == 0 || type == 1)
	{
		return (output / deltaTime);
	}
	else
	{
		return output;
	}

}