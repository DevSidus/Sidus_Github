// File: kalmanFilter.h

#ifndef KALMANFILTER_H
#define KALMANFILTER_H

// Include Files
#include <cmath>
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "kalmanFilter_types.h"

// Function Declarations
extern void kalmanFilter(const double m_n1[3], const double P_n1[9], const
	double y_n[2], const double F[9], const double Q[9], const double H[6], const
	double R[4], double m_n[3], double P_n[9]);

#endif

//
// File trailer for kalmanFilter.h
//
// [EOF]
//
