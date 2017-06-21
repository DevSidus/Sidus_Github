/*
 Filter Coefficients Header File
 Generated after MATLAB(R) Analysis
 */

// Low Pass Filter with Following Parameters
// FilterOrder: 4
// Group Delay: 2 (20ms)
// Cutoff Frequency: 2Hz
const int lpfLength = 5;
const double lpfCoefficient[5] = {
	0.0354109296463815, 0.240923529100402, 0.447331082506433, 0.240923529100402, 0.0354109296463815
};



// Diff Filter with Following Parameters
// FilterOrder: 4
// Group Delay: 2 (20ms)
// PassbandFrequency: 2Hz
// StopbandFrequency: 2.1Hz

const int diffFilterLength = 5;
const double diffFilterCoefficient[5] = {
	0.187871016156492, 0.0942057253992984, 0, -0.0942057253992984, -0.187871016156492
};