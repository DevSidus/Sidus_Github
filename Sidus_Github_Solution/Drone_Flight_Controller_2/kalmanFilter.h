//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
// File: kalmanFilter.h
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 04-Jun-2017 19:49:51
//
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
  double y_n[3], const double F[9], const double Q[9], const double H[9], const
  double R[9], double m_n[3], double P_n[9]);

#endif

//
// File trailer for kalmanFilter.h
//
// [EOF]
//
