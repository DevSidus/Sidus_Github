/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif

#include "Local_PID_v1.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up 
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double* MeasuredVal, double* Output, double* Setpoint)
{
	
    myOutput = Output;
    myMeasuredVal = MeasuredVal;
    mySetpoint = Setpoint;
	inAuto = false;
	d_bypass = 0;
	d_bypass_enabled = false;
	
	PID::SetOutputLimits(-250, 250);				//default output limit corresponds to 
	
    //PID::SetTunings(Kp, Ki, Kd);
	Kp = 1;
	Ki = 0;
	Kd = 0;

	PID::SetMode(AUTOMATIC);
}

PID::PID(double* MeasuredVal, double* Output, double* Setpoint, double* _d_bypass)
{

	myOutput = Output;
	myMeasuredVal = MeasuredVal;
	mySetpoint = Setpoint;
	inAuto = false;
	d_bypass = _d_bypass;
	d_bypass_enabled = true;

	PID::SetOutputLimits(-250, 250);				//default output limit corresponds to 

													//PID::SetTunings(Kp, Ki, Kd);
	Kp = 1;
	Ki = 0;
	Kd = 0;

	PID::SetMode(AUTOMATIC);
}


bool PID::Compute()
{
   if(!inAuto) return false;
   unsigned long now = millis();
   unsigned long dTime = (now - lastTime);

   if(dTime >= 2)
   {
      /*Compute all the working error variables*/

	  double dTimeInSec = dTime / 1000.0;
      double error = *mySetpoint - *myMeasuredVal;

	  errorSmooth = error * (1 - f1) + errorSmooth * f1;




 
      /*Compute PID Output*/

	  //Calculate Proportional Term
	  P_Result = Kp * errorSmooth;

	  if (P_Result > outMax) P_Result = outMax;
	  else if (P_Result < outMin) P_Result = outMin;

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
			  I_Result += (Ki * dTimeInSec * errorSmooth);

			  //We may choose to limit the I term one third of maximum PID output
			  if (I_Result > outMax/3) I_Result = outMax/3;
			  else if (I_Result < outMin/3) I_Result = outMin/3;
		  }
	  }
	  else
	  {
		  I_Result = 0;
	  }
	  
	  //Calculate Derivative Term
	  //bypass value could be inserted just before errorderivativesmooth
	  if (d_bypass_enabled)
	  {
		  errorDerivative = (*mySetpoint - lastSetpoint) / dTimeInSec;
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
      
	  if(output > outMax) output = outMax;
      else if(output < outMin) output = outMin;

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
void PID::SetTunings(double _Kp, double _Ki, double _Kd)
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
void PID::SetOutputLimits(double Min, double Max)
{
   if(Min >= Max) return;
   outMin = Min;
   outMax = Max;
 
   if(inAuto)
   {
	   if(*myOutput > outMax) *myOutput = outMax;
	   else if(*myOutput < outMin) *myOutput = outMin;
	 
	   if(I_Result > outMax) I_Result= outMax;
	   else if(I_Result < outMin) I_Result= outMin;
   }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/ 
void PID::SetMode(int Mode)
{
    bool newAuto = (Mode == AUTOMATIC);
    if(newAuto == !inAuto)
    {  /*we just went from manual to auto*/
        PID::Initialize();
    }
    inAuto = newAuto;
}
 
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/ 
void PID::Initialize()
{
   I_Result = *myOutput;
   if(I_Result > outMax) I_Result = outMax;
   else if(I_Result < outMin) I_Result = outMin;
   
   lastTime = millis();
   lastError = 0;

   f1 = 0;
   f2 = 0.5;
   errorSmooth = 0;
   errorDerivativeSmooth = 0;
   inFlight = false;
   transientInterval = true;
   transientSetpointThreshold = 5;
   transientStartTime = millis();
   transientDuration = 500; //in milliseconds
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display 
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetKp(){ return  Kp; }
double PID::GetKi(){ return  Ki;}
double PID::GetKd(){ return  Kd;}

float PID::GetF1() { return  f1; }
float PID::GetF2() { return  f2; }

void PID::SetKp(double _Kp) { Kp = _Kp; }
void PID::SetKi(double _Ki) { Ki = _Ki; }
void PID::SetKd(double _Kd) { Kd = _Kd; }

void PID::SetF1(float _f1) { f1 = _f1; }
void PID::SetF2(float _f2) { f2 = _f2; }


double PID::Get_P_Result() { return P_Result; }
double PID::Get_I_Result() { return I_Result; }
double PID::Get_D_Result() { return D_Result; }

void PID::Set_I_Result(double val) { I_Result = val; }

int PID::GetMode(){ return  inAuto ? AUTOMATIC : MANUAL;}


void PID::SetFlightMode(bool isInFlight)
{
	inFlight = isInFlight;
}

