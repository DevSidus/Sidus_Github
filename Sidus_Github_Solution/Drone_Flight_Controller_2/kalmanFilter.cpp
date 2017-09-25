// File: kalmanFilter.cpp

// Include Files
#include "rt_nonfinite.h"
#include "kalmanFilter.h"

// Function Declarations

// Kalman Filter for 3 Parameters Estimation with 2 Measurements
// This Kalman Filter is being used for Altitude, Veocity and Acceleration Estimation with Barometer Altitude and Acceleration Measurement
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
void kalmanFilter3State2Measurement(const double m_n1[3], const double P_n1[9], const double y_n[2],
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
// End of Kalman Filter for 3 Parameters Estimation with 2 Measurements


// Kalman Filter for 1 Parameter Estimation with 1 Measurement
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
// Arguments    : double m_n1
//                double P_n1
//                double y_n
//                double F
//                double Q
//                double H
//                double R
//                double *m_n
//                double *P_n
// Return Type  : void
//
void kalmanFilterOneParameter(double m_n1, double P_n1, double y_n, double F, double Q,
	double H, double R, double *m_n, double *P_n)
{
	double m_nn1;
	double P_nn1;
	double K;

	//  Predict
	m_nn1 = F * m_n1;
	P_nn1 = F * P_n1 * F + Q;

	//  Update
	K = P_nn1 * H / (H * P_nn1 * H + R);
	*m_n = m_nn1 + K * (y_n - H * m_nn1);
	*P_n = P_nn1 - K * H * P_nn1;
}
// End of Kalman Filter for 1 Parameter Estimation with 1 Measurement


// Kalman Filter for 3 Parameter Estimation with 1 Measurement
// This Kalman Filter is being used for Heading Estimation with Yaw and Compass Measurement
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
//                double y_n
//                const double F[9]
//                const double Q[9]
//                const double H[3]
//                double R
//                double m_n[3]
//                double P_n[9]
// Return Type  : void
//
void kalmanFilter3State1Measurement(const double m_n1[3], const double P_n1[9], double y_n, const
	double F[9], const double Q[9], const double H[3], double R,
	double m_n[3], double P_n[9])
{
	int i0;
	double d0;
	int i1;
	double b_F[9];
	double S;
	double K[3];
	int i2;
	double P_nn1[9];
	double d1;

	//  Predict
	for (i0 = 0; i0 < 3; i0++) {
		m_n[i0] = 0.0;
		for (i1 = 0; i1 < 3; i1++) {
			b_F[i0 + 3 * i1] = 0.0;
			for (i2 = 0; i2 < 3; i2++) {
				b_F[i0 + 3 * i1] += F[i0 + 3 * i2] * P_n1[i2 + 3 * i1];
			}

			m_n[i0] += F[i0 + 3 * i1] * m_n1[i1];
		}

		for (i1 = 0; i1 < 3; i1++) {
			d0 = 0.0;
			for (i2 = 0; i2 < 3; i2++) {
				d0 += b_F[i0 + 3 * i2] * F[i1 + 3 * i2];
			}

			P_nn1[i0 + 3 * i1] = d0 + Q[i0 + 3 * i1];
		}
	}

	//  Update
	d0 = 0.0;
	for (i0 = 0; i0 < 3; i0++) {
		K[i0] = 0.0;
		for (i1 = 0; i1 < 3; i1++) {
			K[i0] += H[i1] * P_nn1[i1 + 3 * i0];
		}

		d0 += K[i0] * H[i0];
	}

	S = d0 + R;
	d0 = 0.0;
	for (i0 = 0; i0 < 3; i0++) {
		d1 = 0.0;
		for (i1 = 0; i1 < 3; i1++) {
			d1 += P_nn1[i0 + 3 * i1] * H[i1];
		}

		K[i0] = d1 / S;
		d0 += H[i0] * m_n[i0];
	}

	S = y_n - d0;
	for (i0 = 0; i0 < 3; i0++) {
		for (i1 = 0; i1 < 3; i1++) {
			b_F[i0 + 3 * i1] = K[i0] * H[i1];
		}

		for (i1 = 0; i1 < 3; i1++) {
			d0 = 0.0;
			for (i2 = 0; i2 < 3; i2++) {
				d0 += b_F[i0 + 3 * i2] * P_nn1[i2 + 3 * i1];
			}

			P_n[i0 + 3 * i1] = P_nn1[i0 + 3 * i1] - d0;
		}

		m_n[i0] += K[i0] * S;
	}
}
// End of Kalman Filter for 3 Parameter Estimation with 1 Measurement


// Kalman Filter for 3 Parameter Estimation with 3 Measurement
// This Kalman Filter is being used for Position Estimation with GPS and Accelerometer Measurement
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
void kalmanFilter3State3Measurement(const double m_n1[3], const double P_n1[9], const double y_n[3],
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
// End of Kalman Filter for 3 Parameter Estimation with 3 Measurement


//
// File trailer for kalmanFilter.cpp
//
// [EOF]
//