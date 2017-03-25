
#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID_YawAngle.h"

/*Constructor (...)*********************************************************
*    The parameters specified here are those for for which we can't set up
*    reliable defaults, so we need to have the user set them.
***************************************************************************/
PID_YawAngle::PID_YawAngle(double* MeasuredVal, double* Output, double* Setpoint)
{

	myOutput = Output;
	myMeasuredVal = MeasuredVal;
	mySetpoint = Setpoint;
	inAuto = false;
	d_bypass = 0;
	d_bypass_enabled = false;

	PID_YawAngle::SetOutputLimits(-250, 250);				//default output limit corresponds to 

													//PID_YawAngle::SetTunings(Kp, Ki, Kd);
	Kp = 1;
	Ki = 0;
	Kd = 0;

	PID_YawAngle::SetMode(AUTOMATIC);
}

PID_YawAngle::PID_YawAngle(double* MeasuredVal, double* Output, double* Setpoint, double* _d_bypass)
{

	myOutput = Output;
	myMeasuredVal = MeasuredVal;
	mySetpoint = Setpoint;
	inAuto = false;
	d_bypass = _d_bypass;
	d_bypass_enabled = true;

	PID_YawAngle::SetOutputLimits(-250, 250);				//default output limit corresponds to 

													//PID_YawAngle::SetTunings(Kp, Ki, Kd);
	Kp = 1;
	Ki = 0;
	Kd = 0;

	PID_YawAngle::SetMode(AUTOMATIC);
}


bool PID_YawAngle::Compute()
{
	if (!inAuto) return false;
	unsigned long now = millis();
	unsigned long dTime = (now - lastTime);

	if (dTime >= 10)
	{
		/*Compute all the working error variables*/

		double dTimeInSec = dTime / 1000.0;

		//!!! mySetpoint and myMeasuredVal values should be in between -PI and +PI
		double error = *mySetpoint - *myMeasuredVal;

		//Make sure that the error is in between -PI and +PI
		if (error > 180) error -= 360;
		else if (error <= -180) error += 360;

		//Make hysteresis for angles more than 170 degrees in absolute, i.e do not refresh error, use previous
		if (abs(error) > 170)
			error = lastError;


		errorSmooth = error * (1 - f1) + errorSmooth * f1;

		double deltaSetpoint = *mySetpoint - lastSetpoint;
		//Make sure that the deltaSetpoint is in between -PI and +PI
		if (deltaSetpoint > 180) deltaSetpoint -= 360;
		else if (deltaSetpoint <= -180) deltaSetpoint += 360;




		/*Compute PID_YawAngle Output*/

		//Calculate Proportional Term
		P_Result = Kp * errorSmooth;

		//Calculate Integral Term
		if (inFlight)
		{
			//Decide whether the state is transient
			if (abs(*mySetpoint - lastSetpoint) > transientSetpointThreshold)
			{
				transientInterval = true;
				transientStartTime = now;
			}
			else if ((now - transientStartTime) > transientDuration)
			{
				transientInterval = false;
			}

			if (!transientInterval)
			{
				//errorSum -= errorArray[errorArrayIndex];
				//errorSum += errorSmooth;
				//errorArray[errorArrayIndex] = errorSmooth;
				//errorArrayIndex = (errorArrayIndex + 1) % errorArraySize;
				//I_Result = Ki * dTimeInSec * errorSum;
				I_Result += (Ki * dTimeInSec * errorSmooth);

				//We may choose to limit the I term one third of maximum PID output
				if (I_Result > outMax / 3) I_Result = outMax / 3;
				else if (I_Result < outMin / 3) I_Result = outMin / 3;
			}
		}
		else
		{
			errorSum = 0;
			I_Result = 0;
		}

		//Calculate Derivative Term
		//bypass value could be inserted just before errorderivativesmooth
		if (d_bypass_enabled)
		{
			errorDerivative = (deltaSetpoint) / dTimeInSec;
			errorDerivativeSmooth = errorDerivative * (1 - f2) + errorDerivativeSmooth * (f2);
			D_Result = Kd * (errorDerivativeSmooth + (-*d_bypass));
		}
		else
		{
			errorDerivative = (errorSmooth - lastError) / dTimeInSec;
			errorDerivativeSmooth = errorDerivative * (1 - f2) + errorDerivativeSmooth * (f2);

			D_Result = Kd * errorDerivativeSmooth;
		}


		double output = P_Result + I_Result + D_Result;

		if (output > outMax) output = outMax;
		else if (output < outMin) output = outMin;

		*myOutput = output;

		/*Remember some variables for next time*/
		lastTime = now;
		lastError = errorSmooth;
		lastSetpoint = *mySetpoint;
		return true;
	}
	else return false;
}


/* SetTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID_YawAngle::SetTunings(double _Kp, double _Ki, double _Kd)
{
	if (Kp<0 || Ki<0 || Kd<0) return;

	Kp = _Kp;
	Ki = _Ki;
	Kd = _Kd;
}

/* SetOutputLimits(...)****************************************************
*     This function will be used far more often than SetInputLimits.  while
*  the input to the controller will generally be in the 0-1023 range (which is
*  the default already,)  the output will be a little different.  maybe they'll
*  be doing a time window and will need 0-8000 or something.  or maybe they'll
*  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
*  here.
**************************************************************************/
void PID_YawAngle::SetOutputLimits(double Min, double Max)
{
	if (Min >= Max) return;
	outMin = Min;
	outMax = Max;

	if (inAuto)
	{
		if (*myOutput > outMax) *myOutput = outMax;
		else if (*myOutput < outMin) *myOutput = outMin;

		if (I_Result > outMax) I_Result = outMax;
		else if (I_Result < outMin) I_Result = outMin;
	}
}

/* SetMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID_YawAngle::SetMode(int Mode)
{
	bool newAuto = (Mode == AUTOMATIC);
	if (newAuto == !inAuto)
	{  /*we just went from manual to auto*/
		PID_YawAngle::Initialize();
	}
	inAuto = newAuto;
}

/* Initialize()****************************************************************
*	does all the things that need to happen to ensure a bumpless transfer
*  from manual to automatic mode.
******************************************************************************/
void PID_YawAngle::Initialize()
{
	I_Result = *myOutput;
	if (I_Result > outMax) I_Result = outMax;
	else if (I_Result < outMin) I_Result = outMin;

	lastTime = millis();
	lastError = 0;

	f1 = 0;
	f2 = 0.5;
	errorSmooth = 0;
	errorDerivativeSmooth = 0;
	inFlight = false;
	errorArrayIndex = 0;
	errorArray[0] = 0;
	errorSum = 0;
	transientInterval = true;
	transientSetpointThreshold = 5;
	transientStartTime = millis();
	transientDuration = 500; //in milliseconds
}

/* Status Funcions*************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened.  these
* functions query the internal state of the PID_YawAngle.  they're here for display
* purposes.  this are the functions the PID_YawAngle Front-end uses for example
******************************************************************************/
double PID_YawAngle::GetKp() { return  Kp; }
double PID_YawAngle::GetKi() { return  Ki; }
double PID_YawAngle::GetKd() { return  Kd; }

float PID_YawAngle::GetF1() { return  f1; }
float PID_YawAngle::GetF2() { return  f2; }

void PID_YawAngle::SetKp(double _Kp) { Kp = _Kp; }
void PID_YawAngle::SetKi(double _Ki) { Ki = _Ki; }
void PID_YawAngle::SetKd(double _Kd) { Kd = _Kd; }

void PID_YawAngle::SetF1(float _f1) { f1 = _f1; }
void PID_YawAngle::SetF2(float _f2) { f2 = _f2; }


double PID_YawAngle::Get_P_Result() { return P_Result; }
double PID_YawAngle::Get_I_Result() { return I_Result; }
double PID_YawAngle::Get_D_Result() { return D_Result; }

int PID_YawAngle::GetMode() { return  inAuto ? AUTOMATIC : MANUAL; }


void PID_YawAngle::SetFlightMode(bool isInFlight)
{
	inFlight = isInFlight;
}

