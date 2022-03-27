#pragma once
class cDataFilter
{
private:
	int type;
	double coeff[7];
	double data[7];
	double deltaTime;
	int index;
public:
	cDataFilter(int type);
	~cDataFilter();
	double filter(double _val, double _dTime);
};

