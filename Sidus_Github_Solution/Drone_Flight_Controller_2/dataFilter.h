/*
Data Filtering Header File with Filter Coefficients
*/

#include <vector>
using namespace std;

//// Low Pass Filter Coefficients with Following Parameters
//// FilterOrder: 4
//// Group Delay: 2 (20ms)
//// Cutoff Frequency: 2Hz
//const int lpfLength = 5;
//const double lpfCoefficient[5] = {
//	0.0354109296463815, 0.240923529100402, 0.447331082506433, 0.240923529100402, 0.0354109296463815
//};


// Low Pass Filter Coefficients with Following Parameters
// Design Method: Least Squares
// FilterOrder: 6
// Group Delay: 3 (30ms)
// wpass: 0.1
// wstop: 0.15
const int lpfLength = 7;
const double lpfCoefficient[7] = {
	0.120477096721799, 0.145835732118612, 0.162283986090795, 0.167981301398717, 0.162283986090795, 0.145835732118612, 0.120477096721799
};


// Diff Filter Coefficients with Following Parameters
// FilterOrder: 6
// Group Delay: 3 Sample
// PassbandFrequency: 2Hz
// StopbandFrequency: 2.1Hz
// Fs: 200 Hz
const int diffFilter200HzLength = 7;
const double diffFilter200HzCoefficient[7] = {
	-0.101988983017612, -0.0680787959196394, -0.0340652650142217, 0, 0.0340652650142217, 0.0680787959196394, 0.101988983017612
};

// Diff Filter Coefficients with Following Parameters
// FilterOrder: 6
// Group Delay: 3 Sample
// PassbandFrequency: 2Hz
// StopbandFrequency: 2.1Hz
// Fs: 100 Hz
const int diffFilter100HzLength = 7;
const double diffFilter100HzCoefficient[7] = {
	-0.104799329538250, -0.0702839469518364, -0.0352678034184209, 0, 0.0352678034184209, 0.0702839469518364, 0.104799329538250
};

// FIR Filter Function
double dataFilter(vector<double> input, const double filterCoefficient[])
{
	double output = 0.0;

	for (unsigned i = 0; i < input.size(); ++i)
		output = output + input[i] * filterCoefficient[i];

	return output;
}

// Derivative Filter Function
double diffFilter(vector<double> input, const double filterCoefficient[], double deltaTimeSec)
{
	double output = 0.0;

	for (unsigned i = 0; i < input.size(); ++i)
		output = output + input[i] * filterCoefficient[i];

	return output / deltaTimeSec;
}