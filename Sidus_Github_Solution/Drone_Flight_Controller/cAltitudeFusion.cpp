#include "cAltitudeFusion.h"
#include "Arduino.h"

cAltitudeFusion::cAltitudeFusion(double init_pos, double init_vel, double init_acc, double pos_error, double vel_error, double deltaTime)
{

	X_prev[0][0] = init_pos;
	X_prev[1][0] = init_vel;

	A[0][0] = 1;
	A[0][1] = deltaTime;
	A[1][0] = 0;
	A[1][1] = 1;

	B[0][0] = 0.5f*deltaTime * deltaTime;
	B[1][0] = deltaTime;

	U = init_acc;

	P_prev[0][0] = pos_error * pos_error;
	P_prev[0][1] = 0;
	P_prev[1][0] = 0;
	P_prev[1][1] = vel_error * vel_error;

	H[0][0] = 1;
	H[0][1] = 0;

	R[0][0] = pos_error * pos_error;
	Z = 0;
}

cAltitudeFusion::~cAltitudeFusion()
{
}


void cAltitudeFusion::update(double pos, double acc)
{

	//Process Number 1
	X[0][0] = A[0][0] * X_prev[0][0] + A[0][1] * X_prev[1][0] + B[0][0] * U;
	X[1][0] = A[1][0] * X_prev[0][0] + A[1][1] * X_prev[1][0] + B[1][0] * U;


	//Process Number 2&3
	P[0][0] = A[0][0] * P_prev[0][0] + A[0][1] * P_prev[1][0];
	P[0][1] = A[0][0] * P_prev[0][1] + A[0][1] * P_prev[1][1];
	P[1][0] = A[1][0] * P_prev[0][0] + A[1][1] * P_prev[1][0];
	P[1][1] = A[1][0] * P_prev[0][1] + A[1][1] * P_prev[1][1];

	P[0][0] = P[0][0] * A[0][0] + P[0][1] * A[0][1] + 0;
	P[0][1] = P[0][0] * A[1][0] + P[0][1] * A[1][1];
	P[1][0] = P[1][0] * A[0][0] + P[1][1] * A[0][1];
	P[1][1] = P[1][0] * A[1][0] + P[1][1] * A[1][1] + 0.001;

	//Process Number 4

	temp_nom[0][0] = P[0][0] * H[0][0] + P[0][1] * H[0][1];
	temp_nom[1][0] = P[1][0] * H[0][0] + P[1][1] * H[0][1];



	temp_denom[0][0] = H[0][0] * temp_nom[0][0] + H[0][1] * temp_nom[1][0];


	K[0][0] = temp_nom[0][0] / (temp_denom[0][0] + R[0][0]);
	K[1][0] = temp_nom[1][0] / (temp_denom[0][0] + R[0][0]);

	//Process Number 5
	Z = pos;

	//Process Number 6
	double temp = Z - (H[0][0] * X[0][0] + H[0][1] * X[1][0]);


	X[0][0] = X[0][0] + K[0][0] * temp;
	X[1][0] = X[1][0] + K[1][0] * temp;

	//Process Number 7

	temp2x2[0][0] = 1 - K[0][0] * H[0][0];
	temp2x2[0][1] = 0 - K[0][0] * H[0][1];
	temp2x2[1][0] = 0 - K[1][0] * H[0][0];
	temp2x2[1][1] = 1 - K[1][0] * H[0][1];


	P[0][0] = temp2x2[0][0] * P[0][0] + temp2x2[0][1] * P[1][0];
	P[0][1] = temp2x2[0][0] * P[0][1] + temp2x2[0][1] * P[1][1];
	P[1][0] = temp2x2[1][0] * P[0][0] + temp2x2[1][1] * P[1][0];
	P[1][1] = temp2x2[1][0] * P[0][1] + temp2x2[1][1] * P[1][1];


	//Process Number 8
	P_prev[0][0] = P[0][0];
	P_prev[0][1] = P[0][1];
	P_prev[1][0] = P[1][0];
	P_prev[1][1] = P[1][1];

	X_prev[0][0] = X[0][0];
	X_prev[1][0] = X[1][0];

	U = acc;
}