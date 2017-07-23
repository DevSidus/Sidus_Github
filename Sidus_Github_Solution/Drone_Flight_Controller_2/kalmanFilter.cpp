// File: kalmanFilter.cpp

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
//                const double y_n[2]
//                const double F[9]
//                const double Q[9]
//                const double H[6]
//                const double R[4]
//                double m_n[3]
//                double P_n[9]
// Return Type  : void
//
void kalmanFilter(const double m_n1[3], const double P_n1[9], const double y_n[2],
	const double F[9], const double Q[9], const double H[6], const
	double R[4], double m_n[3], double P_n[9])
{
	int r1;
	double m_nn1[3];
	int r2;
	double b_F[9];
	int k;
	double S[4];
	double K[6];
	double a21;
	double y[6];
	double P_nn1[9];
	double a22;
	double b_y_n[2];

	//  Predict
	for (r1 = 0; r1 < 3; r1++) {
		m_nn1[r1] = 0.0;
		for (r2 = 0; r2 < 3; r2++) {
			b_F[r1 + 3 * r2] = 0.0;
			for (k = 0; k < 3; k++) {
				b_F[r1 + 3 * r2] += F[r1 + 3 * k] * P_n1[k + 3 * r2];
			}

			m_nn1[r1] += F[r1 + 3 * r2] * m_n1[r2];
		}

		for (r2 = 0; r2 < 3; r2++) {
			a21 = 0.0;
			for (k = 0; k < 3; k++) {
				a21 += b_F[r1 + 3 * k] * F[r2 + 3 * k];
			}

			P_nn1[r1 + 3 * r2] = a21 + Q[r1 + 3 * r2];
		}
	}

	//  Update
	for (r1 = 0; r1 < 2; r1++) {
		for (r2 = 0; r2 < 3; r2++) {
			K[r1 + (r2 << 1)] = 0.0;
			for (k = 0; k < 3; k++) {
				K[r1 + (r2 << 1)] += H[r1 + (k << 1)] * P_nn1[k + 3 * r2];
			}
		}

		for (r2 = 0; r2 < 2; r2++) {
			a21 = 0.0;
			for (k = 0; k < 3; k++) {
				a21 += K[r1 + (k << 1)] * H[r2 + (k << 1)];
			}

			S[r1 + (r2 << 1)] = a21 + R[r1 + (r2 << 1)];
		}
	}

	for (r1 = 0; r1 < 3; r1++) {
		for (r2 = 0; r2 < 2; r2++) {
			y[r1 + 3 * r2] = 0.0;
			for (k = 0; k < 3; k++) {
				y[r1 + 3 * r2] += P_nn1[r1 + 3 * k] * H[r2 + (k << 1)];
			}
		}
	}

	if (std::abs(S[1]) > std::abs(S[0])) {
		r1 = 1;
		r2 = 0;
	}
	else {
		r1 = 0;
		r2 = 1;
	}

	a21 = S[r2] / S[r1];
	a22 = S[2 + r2] - a21 * S[2 + r1];
	for (k = 0; k < 3; k++) {
		K[k + 3 * r1] = y[k] / S[r1];
		K[k + 3 * r2] = (y[3 + k] - K[k + 3 * r1] * S[2 + r1]) / a22;
		K[k + 3 * r1] -= K[k + 3 * r2] * a21;
	}

	for (r1 = 0; r1 < 2; r1++) {
		a21 = 0.0;
		for (r2 = 0; r2 < 3; r2++) {
			a21 += H[r1 + (r2 << 1)] * m_nn1[r2];
		}

		b_y_n[r1] = y_n[r1] - a21;
	}

	for (r1 = 0; r1 < 3; r1++) {
		a21 = 0.0;
		for (r2 = 0; r2 < 2; r2++) {
			a21 += K[r1 + 3 * r2] * b_y_n[r2];
		}

		m_n[r1] = m_nn1[r1] + a21;
		for (r2 = 0; r2 < 3; r2++) {
			b_F[r1 + 3 * r2] = 0.0;
			for (k = 0; k < 2; k++) {
				b_F[r1 + 3 * r2] += K[r1 + 3 * k] * H[k + (r2 << 1)];
			}
		}

		for (r2 = 0; r2 < 3; r2++) {
			a21 = 0.0;
			for (k = 0; k < 3; k++) {
				a21 += b_F[r1 + 3 * k] * P_nn1[k + 3 * r2];
			}

			P_n[r1 + 3 * r2] = P_nn1[r1 + 3 * r2] - a21;
		}
	}
}

//
// File trailer for kalmanFilter.cpp
//
// [EOF]
//