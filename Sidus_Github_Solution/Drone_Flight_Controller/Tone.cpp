// Tone for Buzzer
// 
// 

#include "Tone.h"
#include "Arduino.h"


void tone_class::playTone(unsigned int toneType)
{
	if (toneType == toneTypeNoTone)
	{
		analogWrite(PIN_BUZZER, 0);
	}
	else if (toneType == toneTypeMotorArmed)
	{
		if (toneArmed[toneIndex])
			analogWrite(PIN_BUZZER, toneArmedDutyCycle);
		else
			analogWrite(PIN_BUZZER, 0);

		toneIndex = (toneIndex + 1) % 20;
	}
	else if (toneType == toneTypeBatteryLow)
	{
		if (toneBatteryLow[toneIndex])
			analogWrite(PIN_BUZZER, toneArmedDutyCycle);
		else
			analogWrite(PIN_BUZZER, 0);

		toneIndex = (toneIndex + 1) % 20;
	}	
}
