
class PID_AccAlt
{


public:

	//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

	//commonly used functions **************************************************************************
	PID_AccAlt(double*, double*, double*);		// * constructor.  links the PID_AccAlt to the Input, Output, and 
										//   Setpoint. 

	PID_AccAlt(double*, double*, double*, double*);		// * constructor.  links the PID_AccAlt to the Input, Output, Setpoint, and 
													//   Derivative of Input. 

	PID_AccAlt(double*, double*, double*, double*, double*);		// * constructor.  links the PID_AccAlt to the Input, Output, Setpoint, and 
															//Derivative of Input, Derivative of Setpoint. 

	void SetMode(int Mode);               // * sets PID_AccAlt to either Manual (0) or Auto (non-0)

	bool Compute();                       // * performs the PID_AccAlt calculation.  it should be
										  //   called every time loop() cycles. ON/OFF and
										  //   calculation frequency can be set using SetMode
										  //   SetSampleTime respectively

	void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
										  //it's likely the user will want to change this depending on
										  //the application



										  //available but not commonly used functions ********************************************************
	void SetTunings(double, double,       // * While most users will set the tunings once in the 
		double);         	  //   constructor, this function gives the user the option
							  //   of changing tunings during runtime for Adaptive control



							  //Display functions ****************************************************************
	double GetKp();						  // These functions query the pid for interal values.
	double GetKi();						  //  they were created mainly for the pid front-end,
	double GetKd();						  // where it's important to know what is actually 

	float GetF1();
	float GetF2();

	void SetKp(double);
	void SetKi(double);
	void SetKd(double);

	void SetF1(float);
	void SetF2(float);

	double Get_P_Result();
	double Get_I_Result();
	double Get_D_Result();

	void Set_I_Result(double);

	int GetMode();						  //  inside the PID_AccAlt.
	void SetFlightMode(bool);			  //For the I term computation enabling

private:
	void Initialize();

	double Kp;                  // * (P)roportional Tuning Parameter
	double Ki;                  // * (I)ntegral Tuning Parameter
	double Kd;                  // * (D)erivative Tuning Parameter

	double dTime;

	double P_Result;
	double I_Result;
	double D_Result;

	double *myMeasuredVal;              // * Pointers to the Input, Output, and Setpoint variables
	double *myOutput;             //   This creates a hard link between the variables and the 
	double *mySetpoint;           //   PID_AccAlt, freeing the user from having to constantly tell us
								  //   what these values are.  with pointers we'll just know.
	double *myMeasuredValDiff;
	double *mySetpointDiff;

	unsigned long lastTime;
	double lastError;
	double errorDerivative;
	double errorSmooth, errorDerivativeSmooth;
	double lastSetpoint;
	float f1, f2;

	double outMin, outMax;
	double range_2;
	bool inAuto;
	bool diff_measuredval_available;
	bool diff_setpoint_available;

	bool inFlight;
	bool transientInterval;
	int transientSetpointThreshold;
	unsigned long transientStartTime;
	unsigned int transientDuration;


};


