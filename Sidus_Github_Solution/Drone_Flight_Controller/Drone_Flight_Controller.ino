/*
 Name:		Drone_Flight_Controller.ino
 Created:	11/10/2016 8:56:34 PM
 Author:	SIDUS
 Description: This is the main code for Drone_Flight_Controller Project
*/


//Use ledc functions for motor pwm usage
#include "esp32-hal-ledc.h"

//Local include class and files
#include "Local_Agenda.h"
#include "Config.h"
#include "cMsgCoWorkerTx.h"
#include "cMsgT01.h"
#include "cMsgR01.h"
#include "cMsgUdpT01.h"
#include "cSerialParse.h"
#include "Local_PID_v1.h"
#include "PID_YawAngle.h"
#include "PID_AccAlt.h"
#include "cRxFilter.h"
#include "cBuzzerMelody.h"


//Global Class Definitions
Agenda scheduler;
cMsgT01 MsgT01;
cMsgR01 MsgR01;
cSerialParse serialParse(sizeof(MsgT01.message), MsgT01.message.startChar1, MsgT01.message.startChar2, MsgT01.message.endChar);
//HardwareSerial Serial2(2);

PID pidRatePitch(&pidVars.ratePitch.sensedVal, &pidVars.ratePitch.output, &pidVars.ratePitch.setpoint);
PID pidAnglePitch(&pidVars.anglePitch.sensedVal, &pidVars.anglePitch.output, &pidVars.anglePitch.setpoint, &pidVars.anglePitch.d_bypass);
PID pidRateRoll(&pidVars.rateRoll.sensedVal, &pidVars.rateRoll.output, &pidVars.rateRoll.setpoint);
PID pidAngleRoll(&pidVars.angleRoll.sensedVal, &pidVars.angleRoll.output, &pidVars.angleRoll.setpoint, &pidVars.angleRoll.d_bypass);
PID pidRateYaw(&pidVars.rateYaw.sensedVal, &pidVars.rateYaw.output, &pidVars.rateYaw.setpoint);
PID_YawAngle pidAngleYaw(&pidVars.angleYaw.sensedVal, &pidVars.angleYaw.output, &pidVars.angleYaw.setpoint, &pidVars.angleYaw.d_bypass);

PID pidVelAlt(&pidVars.velAlt.sensedVal, &pidVars.velAlt.output, &pidVars.velAlt.setpoint, &pidVars.velAlt.d_bypass);
PID_AccAlt pidAccAlt(&pidVars.accAlt.sensedVal, &pidVars.accAlt.output, &pidVars.accAlt.setpoint);

cRxFilter filterRxThr(RX_MAX_PULSE_WIDTH), filterRxPitch(RX_MAX_PULSE_WIDTH), filterRxRoll(RX_MAX_PULSE_WIDTH), filterRxYaw(RX_MAX_PULSE_WIDTH);

cRxFilter filterRx5thCh(RX_MAX_PULSE_WIDTH), filterRx6thCh(RX_MAX_PULSE_WIDTH);



cBuzzerMelody buzzer(PIN_BUZZER, BUZZER_PWM_CHANNEL);

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(SERIAL_COM_SPEED);
	//Serial2.begin(SERIAL_COM_SPEED);
	
	
	//Configure all PINs
	pinMode(PIN_LED, OUTPUT);
	pinMode(PIN_RX_THR, INPUT);
	pinMode(PIN_RX_PITCH, INPUT);
	pinMode(PIN_RX_ROLL, INPUT);
	pinMode(PIN_RX_YAW, INPUT);
	pinMode(PIN_RX_5TH_CHAN, INPUT);
	pinMode(PIN_RX_6TH_CHAN, INPUT);

	pinMode(PIN_BUZZER, OUTPUT);
	
	setupMotorPins();

	attachInterrupt(PIN_RX_THR, isrTHR, CHANGE);
	attachInterrupt(PIN_RX_PITCH, isrPITCH, CHANGE);
	attachInterrupt(PIN_RX_ROLL, isrROLL, CHANGE);
	attachInterrupt(PIN_RX_YAW, isrYAW, CHANGE);

	attachInterrupt(PIN_RX_5TH_CHAN, isrRx5thCh, CHANGE);
	attachInterrupt(PIN_RX_6TH_CHAN, isrRx6thCh, CHANGE);



	//Insert all tasks into scheduler
	scheduler.insert(test_task, 1000000);
	scheduler.insert(mapRxDCtoCmd, 20000);
	scheduler.insert(serialCheck, 8000);
	scheduler.insert(processPID, 9000);
	scheduler.insert(runMotors, 9000);
	scheduler.insert(serialTransmit, 19000);
	scheduler.insert(checkRxStatus, 1000000);
	scheduler.insert(checkMode, 310000);
	scheduler.insert(playMelody, 100000);
	scheduler.insert(handleAutoModeCommands, 100000);

	initVariables();
	initPIDtuning();


}
// the loop function runs over and over again until power down or reset
void loop() {
	//Just call scheduler update and let it do all the process
	scheduler.update();	
}

void initVariables()
{
	modeQuad = modeQuadSAFE;
	autoModeStatus = autoModeOFF;
	statusRx = statusType_NotInitiated;

	startTime_Thr = micros();
	startTime_Pitch = micros();
	startTime_Roll = micros();
	startTime_Yaw = micros();
	startTime_Rx5thCh = micros();
	startTime_Rx6thCh = micros();

	//PID related variable initializations
	pidVars.ratePitch.Kp = PID_RATE_PITCH_KP;
	pidVars.ratePitch.Ki = PID_RATE_PITCH_KI;
	pidVars.ratePitch.Kd = PID_RATE_PITCH_KD;
	pidVars.ratePitch.outputLimitMin = PID_RATE_PITCH_OUTMIN;
	pidVars.ratePitch.outputLimitMax = PID_RATE_PITCH_OUTMAX;
	pidVars.ratePitch.f1 = PID_RATE_PITCH_F1_DEFAULT;
	pidVars.ratePitch.f2 = PID_RATE_PITCH_F2_DEFAULT;
	pidVars.ratePitch.outputFilterConstant = PID_RATE_PITCH_OUT_FILT_CONSTANT;
	pidVars.ratePitch.output = 0;
	pidVars.ratePitch.outputCompensated = 0;

	pidVars.anglePitch.Kp = PID_ANGLE_PITCH_KP;
	pidVars.anglePitch.Ki = PID_ANGLE_PITCH_KI;
	pidVars.anglePitch.Kd = PID_ANGLE_PITCH_KD;
	pidVars.anglePitch.outputLimitMin = PID_ANGLE_PITCH_OUTMIN;
	pidVars.anglePitch.outputLimitMax = PID_ANGLE_PITCH_OUTMAX;
	pidVars.anglePitch.f1 = PID_ANGLE_PITCH_F1_DEFAULT;
	pidVars.anglePitch.f2 = PID_ANGLE_PITCH_F2_DEFAULT;
	pidVars.anglePitch.outputFilterConstant = PID_ANGLE_PITCH_OUT_FILT_CONSTANT;
	pidVars.anglePitch.output = 0;
	pidVars.anglePitch.outputCompensated = 0;

	pidVars.rateRoll.Kp = PID_RATE_ROLL_KP;
	pidVars.rateRoll.Ki = PID_RATE_ROLL_KI;
	pidVars.rateRoll.Kd = PID_RATE_ROLL_KD;
	pidVars.rateRoll.outputLimitMin = PID_RATE_ROLL_OUTMIN;
	pidVars.rateRoll.outputLimitMax = PID_RATE_ROLL_OUTMAX;
	pidVars.rateRoll.f1 = PID_RATE_ROLL_F1_DEFAULT;
	pidVars.rateRoll.f2 = PID_RATE_ROLL_F2_DEFAULT;
	pidVars.rateRoll.outputFilterConstant = PID_RATE_ROLL_OUT_FILT_CONSTANT;
	pidVars.rateRoll.output = 0;
	pidVars.rateRoll.outputCompensated = 0;

	pidVars.angleRoll.Kp = PID_ANGLE_ROLL_KP;
	pidVars.angleRoll.Ki = PID_ANGLE_ROLL_KI;
	pidVars.angleRoll.Kd = PID_ANGLE_ROLL_KD;
	pidVars.angleRoll.outputLimitMin = PID_ANGLE_ROLL_OUTMIN;
	pidVars.angleRoll.outputLimitMax = PID_ANGLE_ROLL_OUTMAX;
	pidVars.angleRoll.f1 = PID_ANGLE_ROLL_F1_DEFAULT;
	pidVars.angleRoll.f2 = PID_ANGLE_ROLL_F2_DEFAULT;
	pidVars.angleRoll.outputFilterConstant = PID_ANGLE_ROLL_OUT_FILT_CONSTANT;
	pidVars.angleRoll.output = 0;
	pidVars.angleRoll.outputCompensated = 0;

	pidVars.rateYaw.Kp = PID_RATE_YAW_KP;
	pidVars.rateYaw.Ki = PID_RATE_YAW_KI;
	pidVars.rateYaw.Kd = PID_RATE_YAW_KD;
	pidVars.rateYaw.outputLimitMin = PID_RATE_YAW_OUTMIN;
	pidVars.rateYaw.outputLimitMax = PID_RATE_YAW_OUTMAX;
	pidVars.rateYaw.f1 = PID_RATE_YAW_F1_DEFAULT;
	pidVars.rateYaw.f2 = PID_RATE_YAW_F2_DEFAULT;
	pidVars.rateYaw.outputFilterConstant = PID_RATE_YAW_OUT_FILT_CONSTANT;
	pidVars.rateYaw.output = 0;
	pidVars.rateYaw.outputCompensated = 0;

	pidVars.angleYaw.Kp = PID_ANGLE_YAW_KP;
	pidVars.angleYaw.Ki = PID_ANGLE_YAW_KI;
	pidVars.angleYaw.Kd = PID_ANGLE_YAW_KD;
	pidVars.angleYaw.outputLimitMin = PID_ANGLE_YAW_OUTMIN;
	pidVars.angleYaw.outputLimitMax = PID_ANGLE_YAW_OUTMAX;
	pidVars.angleYaw.f1 = PID_ANGLE_YAW_F1_DEFAULT;
	pidVars.angleYaw.f2 = PID_ANGLE_YAW_F2_DEFAULT;
	pidVars.angleYaw.outputFilterConstant = PID_ANGLE_YAW_OUT_FILT_CONSTANT;
	pidVars.angleYaw.output = 0;
	pidVars.angleYaw.outputCompensated = 0;


	pidVars.velAlt.Kp = PID_VEL_ALT_KP;
	pidVars.velAlt.Ki = PID_VEL_ALT_KI;
	pidVars.velAlt.Kd = PID_VEL_ALT_KD;
	pidVars.velAlt.outputLimitMin = PID_VEL_ALT_OUTMIN;
	pidVars.velAlt.outputLimitMax = PID_VEL_ALT_OUTMAX;
	pidVars.velAlt.f1 = PID_VEL_ALT_F1_DEFAULT;
	pidVars.velAlt.f2 = PID_VEL_ALT_F2_DEFAULT;
	pidVars.velAlt.outputFilterConstant = PID_VEL_ALT_OUT_FILT_CONSTANT;
	pidVars.velAlt.output = 0;
	pidVars.velAlt.outputCompensated = 0;

	pidVars.accAlt.Kp = PID_ACC_ALT_KP;
	pidVars.accAlt.Ki = PID_ACC_ALT_KI;
	pidVars.accAlt.Kd = PID_ACC_ALT_KD;
	pidVars.accAlt.outputLimitMin = CMD_THR_ARM_START;
	pidVars.accAlt.outputLimitMax = CMD_THR_MAX;
	pidVars.accAlt.f1 = PID_ACC_ALT_F1_DEFAULT;
	pidVars.accAlt.f2 = PID_ACC_ALT_F2_DEFAULT;
	pidVars.accAlt.outputFilterConstant = PID_ACC_ALT_OUT_FILT_CONSTANT;
	pidVars.accAlt.output = 0;
	pidVars.accAlt.outputCompensated = 0;
}

void initPIDtuning()
{
	pidRatePitch.SetOutputLimits(pidVars.ratePitch.outputLimitMin, pidVars.ratePitch.outputLimitMax);
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);
	pidRatePitch.SetF1(pidVars.ratePitch.f1);
	pidRatePitch.SetF2(pidVars.ratePitch.f2);

	pidAnglePitch.SetOutputLimits(pidVars.anglePitch.outputLimitMin, pidVars.anglePitch.outputLimitMax);
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);
	pidAnglePitch.SetF1(pidVars.anglePitch.f1);
	pidAnglePitch.SetF2(pidVars.anglePitch.f2);

	pidRateRoll.SetOutputLimits(pidVars.rateRoll.outputLimitMin, pidVars.rateRoll.outputLimitMax);
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);
	pidRateRoll.SetF1(pidVars.rateRoll.f1);
	pidRateRoll.SetF2(pidVars.rateRoll.f2);

	pidAngleRoll.SetOutputLimits(pidVars.angleRoll.outputLimitMin, pidVars.angleRoll.outputLimitMax);
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);
	pidAngleRoll.SetF1(pidVars.angleRoll.f1);
	pidAngleRoll.SetF2(pidVars.angleRoll.f2);

	pidRateYaw.SetOutputLimits(pidVars.rateYaw.outputLimitMin, pidVars.rateYaw.outputLimitMax);
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);
	pidRateYaw.SetF1(pidVars.rateYaw.f1);
	pidRateYaw.SetF2(pidVars.rateYaw.f2);

	pidAngleYaw.SetOutputLimits(pidVars.angleYaw.outputLimitMin, pidVars.angleYaw.outputLimitMax);
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);
	pidAngleYaw.SetF1(pidVars.angleYaw.f1);
	pidAngleYaw.SetF2(pidVars.angleYaw.f2);


	pidVelAlt.SetOutputLimits(pidVars.velAlt.outputLimitMin, pidVars.velAlt.outputLimitMax);
	pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);
	pidVelAlt.SetF1(pidVars.velAlt.f1);
	pidVelAlt.SetF2(pidVars.velAlt.f2);

	pidAccAlt.SetOutputLimits(pidVars.accAlt.outputLimitMin, pidVars.accAlt.outputLimitMax);
	pidAccAlt.SetTunings(pidVars.accAlt.Kp, pidVars.accAlt.Ki, pidVars.accAlt.Kd);
	pidAccAlt.SetF1(pidVars.accAlt.f1);
	pidAccAlt.SetF2(pidVars.accAlt.f2);
}

void setupMotorPins()
{
	ledcSetup(M_FL_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_FR_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_BR_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcSetup(M_BL_CHANNEL, PWM_FREQ, PWM_DEPTH);
	ledcAttachPin(PIN_M_FL, M_FL_CHANNEL);
	ledcAttachPin(PIN_M_FR, M_FR_CHANNEL);
	ledcAttachPin(PIN_M_BR, M_BR_CHANNEL);
	ledcAttachPin(PIN_M_BL, M_BL_CHANNEL);
}

void pwmMicroSeconds(int _pwm_channel, int _microseconds)
{
	ledcWrite(_pwm_channel, _microseconds*PWM_MICROSECONDS_TO_BITS);
}

float mapping(float input, float inputMin, float inputMax, float outputMin, float outputMax)
{
	float result = 0;

	if (input <= inputMin)
	{
		result = outputMin;
	}
	else if (input >= inputMax)
	{
		result = outputMax;
	}
	else
	{
		result = outputMin + (input - inputMin) / (inputMax - inputMin) * (outputMax - outputMin);
	}
	return result;
}
//Interrupt Service Routine Functions, used for rx pwm inputs
void isrTHR()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_THR) == HIGH)
	{
		startTime_Thr = now_microsec;
		
	}
	else
	{
		//if((micros() - startTime_Thr) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Thr = now_microsec - startTime_Thr;
		rxLastDataTime = millis();  //we need to define this for each isr in order to fully get status of rx
	}
}
void isrPITCH()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_PITCH) == HIGH)
	{
		startTime_Pitch = now_microsec;
	}
	else
	{
		//if ((micros() - startTime_Pitch) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Pitch = now_microsec - startTime_Pitch;
	}
}
void isrROLL()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_ROLL) == HIGH)
	{
		startTime_Roll = now_microsec;
	}
	else
	{
		//if ((micros() - startTime_Roll) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Roll = now_microsec - startTime_Roll;
	}
}
void isrYAW()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_YAW) == HIGH)
	{
		startTime_Yaw = now_microsec;
	}
	else
	{
		//if ((micros() - startTime_Yaw) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Yaw = now_microsec - startTime_Yaw;
	}
}

void isrRx5thCh()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_5TH_CHAN) == HIGH)
	{
		startTime_Rx5thCh = now_microsec;
	}
	else
	{
		//if ((micros() - startTime_Rx5thCh) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Rx5thCh = now_microsec - startTime_Rx5thCh;
	}
}

void isrRx6thCh()
{
	now_microsec = micros();
	if (digitalRead(PIN_RX_6TH_CHAN) == HIGH)
	{
		startTime_Rx6thCh = now_microsec;
	}
	else
	{
		//if ((micros() - startTime_Rx6thCh) < RX_MAX_PULSE_WIDTH)
		dutyCycle_Rx6thCh = now_microsec - startTime_Rx6thCh;
	}
}
	
void test_task()
{
	test_task_counter++;
	if (test_task_counter % 2 == 0)
	{
		digitalWrite(PIN_LED, HIGH);
	}
	else
	{
		digitalWrite(PIN_LED, LOW);
	}
}

void serialCheck()
{
	int numberofbytes = Serial.available();
	if (numberofbytes > 0)
	{

		//If available number of bytes is less than our buffer size, normal case
		if (numberofbytes <= sizeof(MsgT01.message) * SERIAL_PARSE_OVF_MULT)
		{
			unsigned char buffer[sizeof(MsgT01.message) * SERIAL_PARSE_OVF_MULT];
			Serial.readBytes(buffer, numberofbytes);
			serialParse.Push(buffer, numberofbytes);
			if (serialParse.getParsedData(MsgT01.dataBytes, sizeof(MsgT01.message)))
			{
				MsgT01.setPacket();
				updateMessageVariables();
			}

		}
		//Else if buffer overflow, abnormal case
		else
		{
			//Just read it
			unsigned char buffer[sizeof(MsgT01.message) * SERIAL_PARSE_OVF_MULT];
			Serial.readBytes(buffer, sizeof(MsgT01.message) * SERIAL_PARSE_OVF_MULT);
		}

	}
}

void serialTransmit()
{
	MsgR01.message.modeQuad = modeQuad;
	MsgR01.message.autoModeStatus = autoModeStatus;
	MsgR01.message.statusRx = statusRx;
	
	MsgR01.message.rxThrottle = cmdRxThr;
	MsgR01.message.rxPitch = cmdRxPitch;
	MsgR01.message.rxRoll = cmdRxRoll;
	MsgR01.message.rxYaw = cmdRxYaw;

	MsgR01.message.pidRatePitchKp = pidVars.ratePitch.Kp / RESOLUTION_PID_RATE_KP;
	MsgR01.message.pidRatePitchKi = pidVars.ratePitch.Ki / RESOLUTION_PID_RATE_KI;
	MsgR01.message.pidRatePitchKd = pidVars.ratePitch.Kd / RESOLUTION_PID_RATE_KD;	
	MsgR01.message.pidRatePitchOutput = pidVars.ratePitch.output;
	MsgR01.message.pidRatePitchPresult = pidRatePitch.Get_P_Result();
	MsgR01.message.pidRatePitchIresult = pidRatePitch.Get_I_Result();
	MsgR01.message.pidRatePitchDresult = pidRatePitch.Get_D_Result();
	MsgR01.message.pidRatePitchF1 = pidVars.ratePitch.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidRatePitchF2 = pidVars.ratePitch.f2 / RESOLUTION_PID_F;

	MsgR01.message.pidAnglePitchKp = pidVars.anglePitch.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgR01.message.pidAnglePitchKi = pidVars.anglePitch.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgR01.message.pidAnglePitchKd = pidVars.anglePitch.Kd / RESOLUTION_PID_ANGLE_KD;
	MsgR01.message.pidAnglePitchOutput = pidVars.anglePitch.output;
	MsgR01.message.pidAnglePitchPresult = pidAnglePitch.Get_P_Result();
	MsgR01.message.pidAnglePitchIresult = pidAnglePitch.Get_I_Result();
	MsgR01.message.pidAnglePitchDresult = pidAnglePitch.Get_D_Result();
	MsgR01.message.pidAnglePitchF1 = pidVars.anglePitch.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidAnglePitchF2 = pidVars.anglePitch.f2 / RESOLUTION_PID_F;
	MsgR01.message.pidAnglePitchOutFilter = pidVars.anglePitch.outputFilterConstant / RESOLUTION_PID_F;
	
	MsgR01.message.pidRateRollKp = pidVars.rateRoll.Kp / RESOLUTION_PID_RATE_KP;
	MsgR01.message.pidRateRollKi = pidVars.rateRoll.Ki / RESOLUTION_PID_RATE_KI;
	MsgR01.message.pidRateRollKd = pidVars.rateRoll.Kd / RESOLUTION_PID_RATE_KD;
	MsgR01.message.pidRateRollOutput = pidVars.rateRoll.output;
	MsgR01.message.pidRateRollPresult = pidRateRoll.Get_P_Result();
	MsgR01.message.pidRateRollIresult = pidRateRoll.Get_I_Result();
	MsgR01.message.pidRateRollDresult = pidRateRoll.Get_D_Result();
	MsgR01.message.pidRateRollF1 = pidVars.rateRoll.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidRateRollF2 = pidVars.rateRoll.f2 / RESOLUTION_PID_F;

	MsgR01.message.pidAngleRollKp = pidVars.angleRoll.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgR01.message.pidAngleRollKi = pidVars.angleRoll.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgR01.message.pidAngleRollKd = pidVars.angleRoll.Kd / RESOLUTION_PID_ANGLE_KD;
	MsgR01.message.pidAngleRollOutput = pidVars.angleRoll.output;
	MsgR01.message.pidAngleRollPresult = pidAngleRoll.Get_P_Result();
	MsgR01.message.pidAngleRollIresult = pidAngleRoll.Get_I_Result();
	MsgR01.message.pidAngleRollDresult = pidAngleRoll.Get_D_Result();
	MsgR01.message.pidAngleRollF1 = pidVars.angleRoll.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidAngleRollF2 = pidVars.angleRoll.f2 / RESOLUTION_PID_F;
	MsgR01.message.pidAngleRollOutFilter = pidVars.angleRoll.outputFilterConstant / RESOLUTION_PID_F;

	MsgR01.message.pidRateYawKp = pidVars.rateYaw.Kp / RESOLUTION_PID_RATE_KP;
	MsgR01.message.pidRateYawKi = pidVars.rateYaw.Ki / RESOLUTION_PID_RATE_KI;
	MsgR01.message.pidRateYawKd = pidVars.rateYaw.Kd / RESOLUTION_PID_RATE_KD;
	MsgR01.message.pidRateYawOutput = pidVars.rateYaw.output;
	MsgR01.message.pidRateYawPresult = pidRateYaw.Get_P_Result();
	MsgR01.message.pidRateYawIresult = pidRateYaw.Get_I_Result();
	MsgR01.message.pidRateYawDresult = pidRateYaw.Get_D_Result();
	MsgR01.message.pidRateYawF1 = pidVars.rateYaw.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidRateYawF2 = pidVars.rateYaw.f2 / RESOLUTION_PID_F;

	MsgR01.message.pidAngleYawKp = pidVars.angleYaw.Kp / RESOLUTION_PID_ANGLE_KP;
	MsgR01.message.pidAngleYawKi = pidVars.angleYaw.Ki / RESOLUTION_PID_ANGLE_KI;
	MsgR01.message.pidAngleYawKd = pidVars.angleYaw.Kd / RESOLUTION_PID_ANGLE_YAW_KD;
	MsgR01.message.pidAngleYawOutput = pidVars.angleYaw.output;
	MsgR01.message.pidAngleYawPresult = pidAngleYaw.Get_P_Result();
	MsgR01.message.pidAngleYawIresult = pidAngleYaw.Get_I_Result();
	MsgR01.message.pidAngleYawDresult = pidAngleYaw.Get_D_Result();
	MsgR01.message.pidAngleYawF1 = pidVars.angleYaw.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidAngleYawF2 = pidVars.angleYaw.f2 / RESOLUTION_PID_F;
	MsgR01.message.pidAngleYawOutFilter = pidVars.angleYaw.outputFilterConstant / RESOLUTION_PID_F;

	MsgR01.message.commandedYawAngle = cmdMotorYaw;


	MsgR01.message.pidVelAltKp = pidVars.velAlt.Kp / RESOLUTION_PID_VEL_KP;
	MsgR01.message.pidVelAltKi = pidVars.velAlt.Ki / RESOLUTION_PID_VEL_KI;
	MsgR01.message.pidVelAltKd = pidVars.velAlt.Kd / RESOLUTION_PID_VEL_KD;
	MsgR01.message.pidVelAltOutput = pidVars.velAlt.output;

	MsgR01.message.pidVelAltPresult = pidVelAlt.Get_P_Result();
	MsgR01.message.pidVelAltIresult = pidVelAlt.Get_I_Result();
	MsgR01.message.pidVelAltDresult = pidVelAlt.Get_D_Result();


	MsgR01.message.pidVelAltF1 = pidVars.velAlt.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidVelAltF2 = pidVars.velAlt.f2 / RESOLUTION_PID_F;

	MsgR01.message.pidAccAltKp = pidVars.accAlt.Kp / RESOLUTION_PID_POS_KP;
	MsgR01.message.pidAccAltKi = pidVars.accAlt.Ki / RESOLUTION_PID_POS_KI;
	MsgR01.message.pidAccAltKd = pidVars.accAlt.Kd / RESOLUTION_PID_POS_KD;
	MsgR01.message.pidAccAltOutput = pidVars.accAlt.output;

	MsgR01.message.pidAccAltPresult = pidAccAlt.Get_P_Result();
	MsgR01.message.pidAccAltIresult = pidAccAlt.Get_I_Result();
	MsgR01.message.pidAccAltDresult = pidAccAlt.Get_D_Result();

	MsgR01.message.pidAccAltF1 = pidVars.accAlt.f1 / RESOLUTION_PID_F;
	MsgR01.message.pidAccAltF2 = pidVars.accAlt.f2 / RESOLUTION_PID_F;
	MsgR01.message.pidAccAltOutFilter = pidVars.accAlt.outputFilterConstant / RESOLUTION_PID_F;


	MsgR01.getPacket();
	Serial.write(MsgR01.dataBytes, sizeof(MsgR01.dataBytes));
}

void updateMessageVariables()
{

	mpu.euler.psi = MsgT01.message.coWorkerTxPacket.mpuYaw;  //in radians
	mpu.euler.theta = MsgT01.message.coWorkerTxPacket.mpuPitch;  //in radians
	mpu.euler.phi = MsgT01.message.coWorkerTxPacket.mpuRoll;  //in radians

	mpu.gyro.x = MsgT01.message.coWorkerTxPacket.mpuGyroX;  // in deg/sec
	mpu.gyro.y = MsgT01.message.coWorkerTxPacket.mpuGyroY; // in deg/sec
	mpu.gyro.z = MsgT01.message.coWorkerTxPacket.mpuGyroZ; // in deg/sec

	mpu.accel.x = MsgT01.message.coWorkerTxPacket.mpuAccX;
	mpu.accel.y = MsgT01.message.coWorkerTxPacket.mpuAccY;
	mpu.accel.z = MsgT01.message.coWorkerTxPacket.mpuAccZ;

	mpu.accelBody.x = MsgT01.message.coWorkerTxPacket.mpuAccX;
	mpu.accelBody.y = MsgT01.message.coWorkerTxPacket.mpuAccY;
	mpu.accelBody.z = MsgT01.message.coWorkerTxPacket.mpuAccZ;

	mpu.accelWorld.x = MsgT01.message.coWorkerTxPacket.mpuAccWorldX;
	mpu.accelWorld.y = MsgT01.message.coWorkerTxPacket.mpuAccWorldY;
	mpu.accelWorld.z = MsgT01.message.coWorkerTxPacket.mpuAccWorldZ;

	mpu.velWorld.z = MsgT01.message.coWorkerTxPacket.quadVelocityWorldZ;
	mpu.posWorld.z = MsgT01.message.coWorkerTxPacket.quadPositionWorldZ;


	//Serial.print("theta:");
	//Serial.print(MsgT01.message.coWorkerTxPacket.mpuPitch * 180 / M_PI);
	//Serial.print("   phi:");
	//Serial.println(MsgT01.message.coWorkerTxPacket.mpuRoll * 180 / M_PI);

	/// !!! Do not forget to add multiplication to following identities with correct resolution values
	switch (MsgT01.message.udpT01RelayPacket.pidCommandState)
	{
	case pidCommandApplyRatePitchRoll:
		applyPidCommandRatePitchRoll();
		break;

	case pidCommandApplyAnglePitchRoll:
		applyPidCommandAnglePitchRoll();
		break;

	case pidCommandApplyRateYaw:
		applyPidCommandRateYaw();
		break;

	case pidCommandApplyAngleYaw:
		applyPidCommandAngleYaw();
		break;

	case pidCommandApplyAll:
		applyPidCommandRatePitchRoll();
		applyPidCommandAnglePitchRoll();
		applyPidCommandRateYaw();
		applyPidCommandAngleYaw();
		applyPidCommandVelAlt();
		applyPidCommandAccAlt();
		break;

	case pidCommandApplyVelAlt:
		applyPidCommandVelAlt();
		break;

	case pidCommandApplyAccAlt:
		applyPidCommandAccAlt();
		break;
	default:break;
	}

	batteryVoltageInVolts = float(MsgT01.message.coWorkerTxPacket.batteryVoltageInBits) * 0.00336 * (BAT_VOLT_DIV_R1 + BAT_VOLT_DIV_R2) / BAT_VOLT_DIV_R2;

} 

void runMotors()
{

	if (modeQuad == modeQuadARMED)//modeQuad == modeQuadARMED
	{
		if (cmdMotorThr > CMD_THR_ARM_START)
		{
			calculate_pid_thr_batt_scale_factor();

			pidVars.ratePitch.outputCompensated = pidVars.ratePitch.output* PID_THR_BATT_SCALE_FACTOR;
			pidVars.rateRoll.outputCompensated = pidVars.rateRoll.output*PID_THR_BATT_SCALE_FACTOR;
			pidVars.rateYaw.outputCompensated = pidVars.rateYaw.output*PID_THR_BATT_SCALE_FACTOR;
			pidVars.velAlt.outputCompensated = pidVars.velAlt.output*PID_THR_BATT_SCALE_FACTOR;

			pwmMicroSeconds(M_FL_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_FR_CHANNEL, cmdMotorThr + pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BR_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated - pidVars.rateRoll.outputCompensated - pidVars.rateYaw.outputCompensated);
			pwmMicroSeconds(M_BL_CHANNEL, cmdMotorThr - pidVars.ratePitch.outputCompensated + pidVars.rateRoll.outputCompensated + pidVars.rateYaw.outputCompensated);

		
		}
		else
		{
			pwmMicroSeconds(M_FL_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_FR_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_BR_CHANNEL, CMD_THR_MIN);
			pwmMicroSeconds(M_BL_CHANNEL, CMD_THR_MIN);
		}
	}
	else if (modeQuad == modeQuadDirCmd)
	{ 
		pwmMicroSeconds(M_FL_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_FR_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_BR_CHANNEL, cmdRxThr);
		pwmMicroSeconds(M_BL_CHANNEL, cmdRxThr);
	}
	else
	{
		pwmMicroSeconds(M_FL_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_FR_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_BR_CHANNEL, CMD_THR_MIN);
		pwmMicroSeconds(M_BL_CHANNEL, CMD_THR_MIN);
	}

}

void checkRxStatus()
{
	if (millis() - rxLastDataTime > RX_DATATIME_THRESHOLD)
	{
		statusRx = statusType_Fail;
	}
	else
	{
		statusRx = statusType_Normal;
	}
}

void mapRxDCtoCmd()
{
	if (statusRx == statusType_Normal)
	{
		cmdRxPitch = mapping(filterRxPitch.process(dutyCycle_Pitch), DC_PITCH_MIN, DC_PITCH_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);  //filterRxRoll.process(dutyCycle_Pitch)
		cmdRxRoll = mapping(filterRxRoll.process(dutyCycle_Roll), DC_ROLL_MIN, DC_ROLL_MAX, -CMD_RX_PITCH_ROLL_MAX, CMD_RX_PITCH_ROLL_MAX);
		cmdRxYaw = mapping(filterRxYaw.process(dutyCycle_Yaw), DC_YAW_MIN, DC_YAW_MAX, CMD_YAW_MIN, CMD_YAW_MAX);
		cmdRxThr = mapping(filterRxThr.process(dutyCycle_Thr), DC_THR_MIN, DC_THR_MAX, CMD_THR_MIN, CMD_THR_MAX);

		cmdRx5thCh = mapping(filterRx5thCh.process(dutyCycle_Rx5thCh), DC_5TH_CH_MIN, DC_5TH_CH_MAX, -CMD_5TH_CH_MAX, CMD_5TH_CH_MAX);
		cmdRx6thCh = mapping(filterRx6thCh.process(dutyCycle_Rx6thCh), DC_6TH_CH_MIN, DC_6TH_CH_MAX, -CMD_6TH_CH_MAX, CMD_6TH_CH_MAX);



//This below code segment is written to have smoother and much feasible commands for quadcopter
#ifdef COMMAND_CALIBRATION
		if (abs(cmdRxRoll) > 1)
		{
			cmdRxPitchRollAngle = atan(abs(cmdRxPitch / cmdRxRoll));
			cmdRxPitchRollSF = cos(min(cmdRxPitchRollAngle, M_PI_2 - cmdRxPitchRollAngle));
			

			cmdRxPitchRollXfactor = sin(CMD_MAX_ATTITUDE_IN_RADIANS);




			cmdRxRollCalibratedInRad = cmdRxRoll / CMD_RX_PITCH_ROLL_MAX * cmdRxPitchRollSF * cmdRxPitchRollXfactor;
			cmdRxPitchCalibratedInRad = cmdRxPitch / CMD_RX_PITCH_ROLL_MAX * cmdRxPitchRollSF * cmdRxPitchRollXfactor;
			
			cmdRxRollCalibratedInRad = asin(cmdRxRollCalibratedInRad);
			cmdRxPitchCalibratedInRad = asin(cmdRxPitchCalibratedInRad / cos(cmdRxRollCalibratedInRad));


			if (cmdRxPitch >= 0)
				cmdRxPitchCalibrated = abs(cmdRxPitchCalibratedInRad) * 180.0 / M_PI;
			else
				cmdRxPitchCalibrated = -abs(cmdRxPitchCalibratedInRad) * 180.0 / M_PI;

			if (cmdRxRoll >= 0)
				cmdRxRollCalibrated = abs(cmdRxRollCalibratedInRad) * 180.0 / M_PI;
			else
				cmdRxRollCalibrated = -abs(cmdRxRollCalibratedInRad) * 180.0 / M_PI;

		}
		else
		{
			cmdRxPitchCalibrated = cmdRxPitch;
			cmdRxRollCalibrated = cmdRxRoll;

		}
#endif  //COMMAND_CALIBRATION



	}
	else
	{
		cmdRxPitch = 0;
		cmdRxRoll = 0;
		cmdRxYaw = 0;
		cmdRxThr = CMD_THR_MIN;
		cmdRx5thCh = 0;
		cmdRx6thCh = 0;
	}

}

void checkMode()
{
	if (statusRx != statusType_Normal)
	{
		modeQuad = modeQuadSAFE;
		//Serial.println("QUAD is in SAFE Mode Check Rx!");
		return;
	}

	if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw<CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll > CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadARMED;
		//Serial.println("QUAD is ARMED");
	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll < -CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadSAFE;
		//Serial.println("QUAD is in SAFE Mode");

	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll < -CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch<-CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP)
	{
		//Left for spare usage
	}
	else if (cmdRxThr < CMD_THR_MIN + CMD_MODE_CHANGE_THR_GAP && cmdRxYaw < CMD_YAW_MIN + CMD_MODE_CHANGE_ANGLE_GAP && cmdRxRoll>CMD_RX_PITCH_ROLL_MAX - CMD_MODE_CHANGE_ANGLE_GAP && cmdRxPitch<-CMD_RX_PITCH_ROLL_MAX + CMD_MODE_CHANGE_ANGLE_GAP)
	{
		modeQuad = modeQuadDirCmd;
	}

	if (modeQuad == modeQuadARMED && cmdMotorThr > CMD_THR_TAKEOFF)
	{
		pidRatePitch.SetFlightMode(true);
		pidRateRoll.SetFlightMode(true);
		pidRateYaw.SetFlightMode(true);
		pidAnglePitch.SetFlightMode(true);
		pidAngleRoll.SetFlightMode(true);
		pidAngleYaw.SetFlightMode(true);
		pidVelAlt.SetFlightMode(true);
		pidAccAlt.SetFlightMode(true);
	}
	else
	{
		pidRatePitch.SetFlightMode(false);
		pidRateRoll.SetFlightMode(false);
		pidRateYaw.SetFlightMode(false);
		pidAnglePitch.SetFlightMode(false);
		pidAngleRoll.SetFlightMode(false);
		pidAngleYaw.SetFlightMode(false);
		pidVelAlt.SetFlightMode(false);
		pidAccAlt.SetFlightMode(false);
	}
}

void processPID()
{
	prePIDprocesses();

	getBodyToEulerAngularRates();

	//Set bypass values to euler angle rates
	pidVars.angleRoll.d_bypass  = mpu.eulerRate.phi;
	pidVars.anglePitch.d_bypass = mpu.eulerRate.theta;
	pidVars.angleYaw.d_bypass   = mpu.eulerRate.psi;

	//Pitch Roll Yaw Angle PID

	pidVars.angleRoll.setpoint = cmdMotorRoll;
	pidVars.angleRoll.sensedVal = mpu.euler.phi * 180 / M_PI;
	pidAngleRoll.Compute();
	pidVars.angleRoll.outputFiltered = basicFilter(pidVars.angleRoll.output, pidVars.angleRoll.outputFilterConstant, pidVars.angleRoll.outputFiltered);


	pidVars.anglePitch.setpoint = cmdMotorPitch;
	pidVars.anglePitch.sensedVal = mpu.euler.theta * 180 / M_PI;
	pidAnglePitch.Compute();
	pidVars.anglePitch.outputFiltered = basicFilter(pidVars.anglePitch.output, pidVars.anglePitch.outputFilterConstant, pidVars.anglePitch.outputFiltered);
	
	pidVars.angleYaw.setpoint = cmdMotorYaw;
	pidVars.angleYaw.sensedVal = mpu.euler.psi * 180 / M_PI;
	pidAngleYaw.Compute();
	pidVars.angleYaw.outputFiltered = basicFilter(pidVars.angleYaw.output, pidVars.angleYaw.outputFilterConstant, pidVars.angleYaw.outputFiltered);

	//Transform angle pid outputs to body coordinate axis in order to get correct body angular rate setpoints
	transformAnglePIDoutputsToBody();

	//Pitch Roll Yaw Rate PID

	pidVars.ratePitch.setpoint = rateCmd.y;
	pidVars.ratePitch.sensedVal = mpu.gyro.y; 
	pidRatePitch.Compute();


	pidVars.rateRoll.setpoint = rateCmd.x;
	pidVars.rateRoll.sensedVal = mpu.gyro.x;
	pidRateRoll.Compute();


	pidVars.rateYaw.setpoint = rateCmd.z;
	pidVars.rateYaw.sensedVal = mpu.gyro.z;
	pidRateYaw.Compute();
	

	pidVars.velAlt.d_bypass = mpu.accelWorld.z / 8.36;  // cm/second^2
	pidVars.velAlt.setpoint = cmdRx6thCh;
	pidVars.velAlt.sensedVal = mpu.velWorld.z;  // cm/second
	pidVelAlt.Compute();
	pidVars.velAlt.outputFiltered = basicFilter(pidVars.velAlt.output, pidVars.velAlt.outputFilterConstant, pidVars.velAlt.outputFiltered);

	
	//Transform vel pid outputs to body coordinate axis in order to get correct acceleration wrt world
	transformVelPIDoutputsToBody();

	pidVars.accAlt.setpoint = accelCmd.z;   // cmd/second^2
	pidVars.accAlt.sensedVal = mpu.accel.z / 8.3;  // cm/second^2
	pidAccAlt.Compute();
	
	postPIDprocesses();


}

void playMelody()
{
	if (modeQuad == modeQuadARMED && cmdMotorThr<=CMD_THR_ARM_START)
		buzzer.play(buzzerMelodyArmWarning);
	else
		buzzer.play(buzzerMelodyNoTone);	
}

float basicFilter(float data, float filterConstant, float filteredData)
{
	if (filterConstant > 1) {      // check to make sure param's are within range
		filterConstant = .99;
	}
	else if (filterConstant <= 0) {
		filterConstant = 0;
	}
	filteredData = (data * (1 - filterConstant)) + (filteredData  *  filterConstant);
	return filteredData;
}

void calculate_pid_thr_batt_scale_factor() //this function will be modified to include battery voltage compensation also
{
	float total_thr_cmd = cmdMotorThr + pidVars.velAlt.outputCompensated;   // compensated output or normal output, will be discussed later

	if (total_thr_cmd <= CMD_THR_MAX && total_thr_cmd >= CMD_THR_MIN)
		PID_THR_BATT_SCALE_FACTOR = (CMD_THR_MAX - total_thr_cmd) / (CMD_THR_MAX - CMD_THR_MIN) + 0.2;
	else
		PID_THR_BATT_SCALE_FACTOR = 0.4;

	//Use linear interpolation for battery voltage level compensation
	//Make sure that voltage read is in valid range of operation
	if (batteryVoltageInVolts > 9.0 && batteryVoltageInVolts < 13.2)
	{
		PID_THR_BATT_SCALE_FACTOR = PID_THR_BATT_SCALE_FACTOR * 12.0 / batteryVoltageInVolts;
	}
}

void applyPidCommandRatePitchRoll()
{
	//Set Pitch Rate PID parameters
	pidVars.ratePitch.Kp = MsgT01.message.udpT01RelayPacket.pidRatePitchRollKp * RESOLUTION_PID_RATE_KP;
	pidVars.ratePitch.Ki = MsgT01.message.udpT01RelayPacket.pidRatePitchRollKi * RESOLUTION_PID_RATE_KI;
	pidVars.ratePitch.Kd = MsgT01.message.udpT01RelayPacket.pidRatePitchRollKd * RESOLUTION_PID_RATE_KD;
	pidVars.ratePitch.f1 = MsgT01.message.udpT01RelayPacket.pidRatePitchRollF1 * RESOLUTION_PID_F;
	pidVars.ratePitch.f2 = MsgT01.message.udpT01RelayPacket.pidRatePitchRollF2 * RESOLUTION_PID_F;
	pidRatePitch.SetTunings(pidVars.ratePitch.Kp, pidVars.ratePitch.Ki, pidVars.ratePitch.Kd);
	pidRatePitch.SetF1(pidVars.ratePitch.f1);
	pidRatePitch.SetF2(pidVars.ratePitch.f2);
	//Set Roll Rate PID parameters
	pidVars.rateRoll.Kp = pidVars.ratePitch.Kp;
	pidVars.rateRoll.Ki = pidVars.ratePitch.Ki;
	pidVars.rateRoll.Kd = pidVars.ratePitch.Kd;
	pidVars.rateRoll.f1 = pidVars.ratePitch.f1;
	pidVars.rateRoll.f2 = pidVars.ratePitch.f2;
	pidRateRoll.SetTunings(pidVars.rateRoll.Kp, pidVars.rateRoll.Ki, pidVars.rateRoll.Kd);
	pidRateRoll.SetF1(pidVars.rateRoll.f1);
	pidRateRoll.SetF2(pidVars.rateRoll.f2);
}

void applyPidCommandAnglePitchRoll()
{
	//Set Pitch Angle PID parameters
	pidVars.anglePitch.Kp = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.anglePitch.Ki = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.anglePitch.Kd = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollKd * RESOLUTION_PID_ANGLE_KD;
	pidVars.anglePitch.f1 = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollF1 * RESOLUTION_PID_F;
	pidVars.anglePitch.f2 = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollF2 * RESOLUTION_PID_F;
	pidVars.anglePitch.outputFilterConstant = MsgT01.message.udpT01RelayPacket.pidAnglePitchRollOutFilter * RESOLUTION_PID_F;
	pidVars.angleRoll.outputFilterConstant = pidVars.anglePitch.outputFilterConstant;
	pidAnglePitch.SetTunings(pidVars.anglePitch.Kp, pidVars.anglePitch.Ki, pidVars.anglePitch.Kd);
	pidAnglePitch.SetF1(pidVars.anglePitch.f1);
	pidAnglePitch.SetF2(pidVars.anglePitch.f2);
	//Set Roll Angle PID parameters
	pidVars.angleRoll.Kp = pidVars.anglePitch.Kp;
	pidVars.angleRoll.Ki = pidVars.anglePitch.Ki;
	pidVars.angleRoll.Kd = pidVars.anglePitch.Kd;
	pidVars.angleRoll.f1 = pidVars.anglePitch.f1;
	pidVars.angleRoll.f2 = pidVars.anglePitch.f2;
	pidAngleRoll.SetTunings(pidVars.angleRoll.Kp, pidVars.angleRoll.Ki, pidVars.angleRoll.Kd);
	pidAngleRoll.SetF1(pidVars.angleRoll.f1);
	pidAngleRoll.SetF2(pidVars.angleRoll.f2);
}

void applyPidCommandRateYaw()
{
	//Set Yaw Rate PID parameters
	pidVars.rateYaw.Kp = MsgT01.message.udpT01RelayPacket.pidRateYawKp * RESOLUTION_PID_RATE_KP;
	pidVars.rateYaw.Ki = MsgT01.message.udpT01RelayPacket.pidRateYawKi * RESOLUTION_PID_RATE_KI;
	pidVars.rateYaw.Kd = MsgT01.message.udpT01RelayPacket.pidRateYawKd * RESOLUTION_PID_RATE_KD;
	pidVars.rateYaw.f1 = MsgT01.message.udpT01RelayPacket.pidRateYawF1 * RESOLUTION_PID_F;
	pidVars.rateYaw.f2 = MsgT01.message.udpT01RelayPacket.pidRateYawF2 * RESOLUTION_PID_F;
	pidRateYaw.SetTunings(pidVars.rateYaw.Kp, pidVars.rateYaw.Ki, pidVars.rateYaw.Kd);
	pidRateYaw.SetF1(pidVars.rateYaw.f1);
	pidRateYaw.SetF2(pidVars.rateYaw.f2);
}

void applyPidCommandAngleYaw()
{
	//Set Yaw Angle PID parameters
	pidVars.angleYaw.Kp = MsgT01.message.udpT01RelayPacket.pidAngleYawKp * RESOLUTION_PID_ANGLE_KP;
	pidVars.angleYaw.Ki = MsgT01.message.udpT01RelayPacket.pidAngleYawKi * RESOLUTION_PID_ANGLE_KI;
	pidVars.angleYaw.Kd = MsgT01.message.udpT01RelayPacket.pidAngleYawKd * RESOLUTION_PID_ANGLE_YAW_KD;
	pidVars.angleYaw.f1 = MsgT01.message.udpT01RelayPacket.pidAngleYawF1 * RESOLUTION_PID_F;
	pidVars.angleYaw.f2 = MsgT01.message.udpT01RelayPacket.pidAngleYawF2 * RESOLUTION_PID_F;
	pidVars.angleYaw.outputFilterConstant = MsgT01.message.udpT01RelayPacket.pidAngleYawOutFilter * RESOLUTION_PID_F;
	pidAngleYaw.SetTunings(pidVars.angleYaw.Kp, pidVars.angleYaw.Ki, pidVars.angleYaw.Kd);
	pidAngleYaw.SetF1(pidVars.angleYaw.f1);
	pidAngleYaw.SetF2(pidVars.angleYaw.f2);
}

void applyPidCommandVelAlt()
{
	//Set Altitude Velocity PID parameters
	pidVars.velAlt.Kp = MsgT01.message.udpT01RelayPacket.pidVelAltKp * RESOLUTION_PID_VEL_KP;
	pidVars.velAlt.Ki = MsgT01.message.udpT01RelayPacket.pidVelAltKi * RESOLUTION_PID_VEL_KI;
	pidVars.velAlt.Kd = MsgT01.message.udpT01RelayPacket.pidVelAltKd * RESOLUTION_PID_VEL_KD;
	pidVars.velAlt.f1 = MsgT01.message.udpT01RelayPacket.pidVelAltF1 * RESOLUTION_PID_F;
	pidVars.velAlt.f2 = MsgT01.message.udpT01RelayPacket.pidVelAltF2 * RESOLUTION_PID_F;
	pidVars.velAlt.outputFilterConstant = MsgT01.message.udpT01RelayPacket.pidVelAltOutFilter * RESOLUTION_PID_F;
	pidVelAlt.SetTunings(pidVars.velAlt.Kp, pidVars.velAlt.Ki, pidVars.velAlt.Kd);
	pidVelAlt.SetF1(pidVars.velAlt.f1);
	pidVelAlt.SetF2(pidVars.velAlt.f2);
}

void applyPidCommandAccAlt()
{
	//Set Altitude Position PID parameters
	pidVars.accAlt.Kp = MsgT01.message.udpT01RelayPacket.pidAccAltKp * RESOLUTION_PID_POS_KP;
	pidVars.accAlt.Ki = MsgT01.message.udpT01RelayPacket.pidAccAltKi * RESOLUTION_PID_POS_KI;
	pidVars.accAlt.Kd = MsgT01.message.udpT01RelayPacket.pidAccAltKd * RESOLUTION_PID_POS_KD;
	pidVars.accAlt.f1 = MsgT01.message.udpT01RelayPacket.pidAccAltF1 * RESOLUTION_PID_F;
	pidVars.accAlt.f2 = MsgT01.message.udpT01RelayPacket.pidAccAltF2 * RESOLUTION_PID_F;
	pidAccAlt.SetTunings(pidVars.accAlt.Kp, pidVars.accAlt.Ki, pidVars.accAlt.Kd);
	pidAccAlt.SetF1(pidVars.accAlt.f1);
	pidAccAlt.SetF2(pidVars.accAlt.f2);
}

void prePIDprocesses()
{
	cmdMotorPitch = cmdRxPitchCalibrated;
	cmdMotorRoll = cmdRxRollCalibrated;

	

	if (modeQuad == modeQuadARMED && cmdMotorThr > CMD_THR_TAKEOFF)
	{
		if (abs(cmdRxYaw) > 6)
		{
			cmdMotorYaw += cmdRxYaw / 60.0;

			if (cmdMotorYaw > 180)
				cmdMotorYaw -= 360;
			else if (cmdMotorYaw <= -180)
				cmdMotorYaw += 360;
		}
	}
	else
	{
		cmdMotorYaw = mpu.euler.psi * 180 / M_PI;
	}
}

void postPIDprocesses()
{
	if (autoModeStatus == autoModeAltitude)
	{
		cmdMotorThr = pidVars.accAlt.output;
	}
	else if (autoModeStatus == autoModeAltToOFF)
	{
		if (abs(cmdMotorThr - cmdRxThr) < 4)
		{
			autoModeStatus = autoModeOFF;
		}
		else if (cmdMotorThr > cmdRxThr)
		{
			cmdMotorThr -= 0.2;
		}
		else
		{
			cmdMotorThr += 0.2;
		}
	}
	else
	{
		cmdMotorThr = cmdRxThr;
		pidAccAlt.Set_I_Result(cmdMotorThr);
	}

}

void handleAutoModeCommands()
{
#ifdef MY_RX_TX_IS_6_CHANNEL
	if (modeQuad == modeQuadARMED && (statusRx == statusType_Normal) && (cmdRx5thCh > (CMD_5TH_CH_MAX-10)))
	{
		autoModeStatus = autoModeAltitude;
	}
	else if(autoModeStatus != autoModeOFF)
	{
		//If previous autoModeStatus is Altitude, then change to transition state
		autoModeStatus = autoModeAltToOFF;
	}	
#else	
	if (modeQuad == modeQuadARMED && MsgT01.message.coWorkerTxPacket.statusGS == statusType_Normal)
	{
		autoModeStatus = MsgT01.message.udpT01RelayPacket.autoModeCommand;
	}
	else
	{
		autoModeStatus = autoModeOFF;
	}

#endif // MY_RX_TX_IS_6_CHANNEL

}

void getBodyToEulerAngularRates()
{

	if (abs(cos(mpu.euler.theta)) > 0.0001)  
	{
		mpu.eulerRate.phi = mpu.gyro.x + sin(mpu.euler.phi) * tan(mpu.euler.theta) * mpu.gyro.y + cos(mpu.euler.phi) * tan(mpu.euler.theta) * mpu.gyro.z;
		mpu.eulerRate.theta = cos(mpu.euler.phi)*mpu.gyro.y - sin(mpu.euler.phi)*mpu.gyro.z;
		mpu.eulerRate.psi = sin(mpu.euler.phi) / cos(mpu.euler.theta) * mpu.gyro.y + cos(mpu.euler.phi) / cos(mpu.euler.theta)*mpu.gyro.z;
	}


}

void transformAnglePIDoutputsToBody()
{
	rateCmd.x = pidVars.angleRoll.outputFiltered - sin(mpu.euler.theta) * pidVars.angleYaw.outputFiltered;
	rateCmd.y = cos(mpu.euler.phi) * pidVars.anglePitch.outputFiltered + sin(mpu.euler.phi)*cos(mpu.euler.theta)*pidVars.angleYaw.outputFiltered;
	rateCmd.z = -sin(mpu.euler.phi) * pidVars.anglePitch.outputFiltered + cos(mpu.euler.phi)*cos(mpu.euler.theta)*pidVars.angleYaw.outputFiltered;


}

void transformVelPIDoutputsToBody()
{

	//this code will be modified according to the behaviour of quadcopter during flight tests
	//double tempVar = cos(mpu.euler.phi)*cos(mpu.euler.theta);
	//if (abs(tempVar) > 0.5)
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / tempVar) + 500;
	//}
	//else if (tempVar >= 0)
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / 0.5) + 500;
	//}
	//else
	//{
	//	accelCmd.z = ((pidVars.velAlt.outputFiltered + 500) / -0.5) -500;   //this condition should be discussed
	//}

	accelCmd.z = (pidVars.velAlt.outputFiltered + 1000);

}