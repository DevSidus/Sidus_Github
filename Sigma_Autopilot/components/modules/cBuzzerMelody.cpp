#include "cBuzzerMelody.h"

cBuzzerMelody::cBuzzerMelody(unsigned short _buzzer_pin, unsigned short _buzzer_pwm_channel)
{
	buzzer_pin = _buzzer_pin;
	buzzer_pwm_channel = _buzzer_pwm_channel;
	unsigned short toneIndex = 0;

	//frequency:2000Hz 16 bit resolution
	ledcSetup(buzzer_pwm_channel, 2000, 16);
	ledcAttachPin(buzzer_pin, buzzer_pwm_channel);
	dutyCycle_In_Bits = 32000;
}


cBuzzerMelody::~cBuzzerMelody()
{
}

void cBuzzerMelody::play(unsigned short _type)
{
	if (_type == 0)
	{
		if (toneSafe[toneIndex])
			ledcWrite(buzzer_pwm_channel, dutyCycle_In_Bits);
		else
			ledcWrite(buzzer_pwm_channel, 0);

		toneIndex = (toneIndex + 1) % 20;
	}
	else if (_type == 1)
	{
		if (toneArmed[toneIndex])
			ledcWrite(buzzer_pwm_channel, dutyCycle_In_Bits);
		else
			ledcWrite(buzzer_pwm_channel, 0);

		toneIndex = (toneIndex + 1) % 20;
	}
	else if (_type == 2)
	{
		if (toneDirCmd[toneIndex])
			ledcWrite(buzzer_pwm_channel, dutyCycle_In_Bits);
		else
			ledcWrite(buzzer_pwm_channel, 0);
		toneIndex = (toneIndex + 1) % 20;
	}
	else if (_type == 3)
	{
		if (toneBatteryLow[toneIndex])
			ledcWrite(buzzer_pwm_channel, dutyCycle_In_Bits);
		else
			ledcWrite(buzzer_pwm_channel, 0);
		toneIndex = (toneIndex + 1) % 20;
	}
}
