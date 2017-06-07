/*
* File: kalmanFilter.cpp
*
* MATLAB Coder version            : 3.3
* C/C++ source code generated on  : 04-Jun-2017 19:49:51
*/

// Include Files
#include "rt_nonfinite.h"
#include "kalmanFilter.h"

// Function Definitions

//
// Kalman filter prediction and update steps. propagates Gaussian
//  posterior state distribution from time step n-1 to time step n
//
//  Inputs:
//  m_n1 - (Dx1 vector) Gaussian posterior mean at time step n-1
//  P_n1 - (DxD matrix) Gaussian posterior covariance at time step n-1
//  y_n - (Mx1 vector) measurements at time step n
//  F - (DxD matrix) state-transition matrix at time step n
//  Q - (DxD matrix) Gaussian state noise covariance at time step n
//  H - (MxD matrix) measurement matrix at time step n
//  R - (MxM matrix) Gaussian measurement noise covariance at time step n
//
//  Outputs:
//  m_n - (Dx1 vector) Gaussian posterior mean at time step n
//  P_n - (DxD matrix) Gaussian posterior covariance at time step n
// Arguments    : const double m_n1[3]
//                const double P_n1[9]
//                const double y_n[3]
//                const double F[9]
//                const double Q[9]
//                const double H[9]
//                const double R[9]
//                double m_n[3]
//                double P_n[9]
// Return Type  : void
//
void kalmanFilter(const double m_n1[3], const double P_n1[9], const double y_n[3],
                  const double F[9], const double Q[9], const double H[9], const
                  double R[9], double m_n[3], double P_n[9])
{
  int i0;
  double m_nn1[3];
  int rtemp;
  int r1;
  double b_F[9];
  int r2;
  int k;
  int r3;
  double maxval;
  double S[9];
  double a21;
  double P_nn1[9];
  double y[9];
  double K[9];
  double b_y_n[3];

  //  Predict
  for (i0 = 0; i0 < 3; i0++) {
    m_nn1[i0] = 0.0;
    for (rtemp = 0; rtemp < 3; rtemp++) {
      b_F[i0 + 3 * rtemp] = 0.0;
      for (k = 0; k < 3; k++) {
        b_F[i0 + 3 * rtemp] += F[i0 + 3 * k] * P_n1[k + 3 * rtemp];
      }

      m_nn1[i0] += F[i0 + 3 * rtemp] * m_n1[rtemp];
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval = 0.0;
      for (k = 0; k < 3; k++) {
        maxval += b_F[i0 + 3 * k] * F[rtemp + 3 * k];
      }

      P_nn1[i0 + 3 * rtemp] = maxval + Q[i0 + 3 * rtemp];
    }
  }

  //  Update
  for (i0 = 0; i0 < 3; i0++) {
    for (rtemp = 0; rtemp < 3; rtemp++) {
      b_F[i0 + 3 * rtemp] = 0.0;
      for (k = 0; k < 3; k++) {
        b_F[i0 + 3 * rtemp] += H[i0 + 3 * k] * P_nn1[k + 3 * rtemp];
      }
    }

    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval = 0.0;
      for (k = 0; k < 3; k++) {
        maxval += b_F[i0 + 3 * k] * H[rtemp + 3 * k];
      }

      S[i0 + 3 * rtemp] = maxval + R[i0 + 3 * rtemp];
      y[i0 + 3 * rtemp] = 0.0;
      for (k = 0; k < 3; k++) {
        y[i0 + 3 * rtemp] += P_nn1[i0 + 3 * k] * H[rtemp + 3 * k];
      }
    }
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(S[0]);
  a21 = std::abs(S[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if (std::abs(S[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  S[r2] /= S[r1];
  S[r3] /= S[r1];
  S[3 + r2] -= S[r2] * S[3 + r1];
  S[3 + r3] -= S[r3] * S[3 + r1];
  S[6 + r2] -= S[r2] * S[6 + r1];
  S[6 + r3] -= S[r3] * S[6 + r1];
  if (std::abs(S[3 + r3]) > std::abs(S[3 + r2])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }

  S[3 + r3] /= S[3 + r2];
  S[6 + r3] -= S[3 + r3] * S[6 + r2];
  for (k = 0; k < 3; k++) {
    K[k + 3 * r1] = y[k] / S[r1];
    K[k + 3 * r2] = y[3 + k] - K[k + 3 * r1] * S[3 + r1];
    K[k + 3 * r3] = y[6 + k] - K[k + 3 * r1] * S[6 + r1];
    K[k + 3 * r2] /= S[3 + r2];
    K[k + 3 * r3] -= K[k + 3 * r2] * S[6 + r2];
    K[k + 3 * r3] /= S[6 + r3];
    K[k + 3 * r2] -= K[k + 3 * r3] * S[3 + r3];
    K[k + 3 * r1] -= K[k + 3 * r3] * S[r3];
    K[k + 3 * r1] -= K[k + 3 * r2] * S[r2];
    maxval = 0.0;
    for (i0 = 0; i0 < 3; i0++) {
      maxval += H[k + 3 * i0] * m_nn1[i0];
    }

    b_y_n[k] = y_n[k] - maxval;
  }

  for (i0 = 0; i0 < 3; i0++) {
    maxval = 0.0;
    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval += K[i0 + 3 * rtemp] * b_y_n[rtemp];
      b_F[i0 + 3 * rtemp] = 0.0;
      for (k = 0; k < 3; k++) {
        b_F[i0 + 3 * rtemp] += K[i0 + 3 * k] * H[k + 3 * rtemp];
      }
    }

    m_n[i0] = m_nn1[i0] + maxval;
    for (rtemp = 0; rtemp < 3; rtemp++) {
      maxval = 0.0;
      for (k = 0; k < 3; k++) {
        maxval += b_F[i0 + 3 * k] * P_nn1[k + 3 * rtemp];
      }

      P_n[i0 + 3 * rtemp] = P_nn1[i0 + 3 * rtemp] - maxval;
    }
  }
}

//
// File trailer for kalmanFilter.cpp
//
// [EOF]
//
