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
extern void kalmanFilter(const double m_n1[3], const double P_n1[9], const
	double y_n[2], const double F[9], const double Q[9], const double H[6], const
	double R[4], double m_n[3], double P_n[9]);

// Kalman Filter for 1 Parameter Estimation with 1 Measurement
extern void kalmanFilterOneParameter(double m_n1, double P_n1, double y_n, double F, double
	Q, double H, double R, double *m_n, double *P_n);

// Kalman Filter for 2 Parameter Estimation with 2 Measurement
extern void kalmanFilterAngleEstimation(const double m_n1[2], const double P_n1[4], const
	double y_n[2], const double F[4], const double Q[4], const double H[4], const
	double R[4], double m_n[2], double P_n[4]);

#endif
//
// File trailer for kalmanFilter.h
//
// [EOF]
//
