// File: kalmanFilter.h

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"

// Function Declarations

// Kalman Filter for 3 Parameters Estimation with 2 Measurements
extern void kalmanFilter3State2Measurement(const double m_n1[3], const double P_n1[9], const
	double y_n[2], const double F[9], const double Q[9], const double H[6], const
	double R[4], double m_n[3], double P_n[9]);

// Kalman Filter for 1 Parameter Estimation with 1 Measurement
extern void kalmanFilterOneParameter(double m_n1, double P_n1, double y_n, double F, double
	Q, double H, double R, double *m_n, double *P_n);

// Kalman Filter for 3 Parameter Estimation with 1 Measurement
extern void kalmanFilter3State1Measurement(const double m_n1[3], const double P_n1[9], double y_n,
	const double F[9], const double Q[9], const double H[3], double R, double m_n
	[3], double P_n[9]);

// Kalman Filter for 3 Parameter Estimation with 3 Measurement
extern void kalmanFilter3State3Measurement(const double m_n1[3], const double P_n1[9], const
	double y_n[3], const double F[9], const double Q[9], const double H[9], const
	double R[9], double m_n[3], double P_n[9]);

#endif
//
// File trailer for kalmanFilter.h
//
// [EOF]
//
