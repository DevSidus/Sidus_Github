#pragma once
class cAltitudeFusion
{
public:
	//Constructors
	cAltitudeFusion(double init_pos, double init_vel, double init_acc, double pos_error, double vel_error, double deltaTime);
	~cAltitudeFusion();

	double X_prev[2][1];
	double X[2][1];
	double P_prev[2][2];
	double P[2][2];
	double U;
	double K[2][1];
	double H[1][2];
	double B[2][1];
	double A[2][2];
	double R[1][1];
	double Z;
	void update(double pos, double acc);

private:
	double temp_nom[2][1];
	double temp_denom[1][1];
	double temp2x2[2][2];

};

