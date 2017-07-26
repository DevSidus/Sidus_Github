class PID_YawAngle
{

public:

	//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

	//commonly used functions **************************************************************************
	PID_YawAngle(double*, double*, double*);		// * constructor.  links the PID to the Input, Output, and 
													//   Setpoint. 
	
	PID_YawAngle(double*, double*, double*, double*);		// * constructor.  links the PID to the Input, Output, Setpoint, and 
															//   Derivative of Input. 
	
	PID_YawAngle(double*, double*, double*, double*, double*);		// * constructor.  links the PID to the Input, Output, Setpoint, and 
															// Derivative of Input, Derivative of Setpoint.

	void SetMode(int Mode);               // * sets PID to either Manual (0) or Auto (non-0)

	bool Compute();                       // * performs the PID calculation.  it should be
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
	double GetKp();						  // These functions query the PID for interal values.
	double GetKi();						  //  they were created mainly for the PID front-end,
	double GetKd();						  // where it's important to know what is actually 


	void SetKp(double);
	void SetKi(double);
	void SetKd(double);
	
	double Get_P_Result();
	double Get_I_Result();
	double Get_D_Result();

	int GetMode();						  //  inside the PID.
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
	double *mySetpoint;           //   PID_YawAngle, freeing the user from having to constantly tell us
								  //   what these values are.  with pointers we'll just know.
	double *myMeasuredValDiff;
	double *mySetpointDiff;

	unsigned long lastTime;
	double lastError;
	double errorDerivative;
	double lastSetpoint;

	double outMin, outMax;
	bool inAuto;
	bool diff_measuredval_available;
	bool diff_setpoint_available;

	bool inFlight;


};

