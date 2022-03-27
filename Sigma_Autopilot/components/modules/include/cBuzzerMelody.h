
#pragma once

//Use ledc functions for pwm usage
#include "Arduino.h"
#include "esp32-hal-ledc.h"

class cBuzzerMelody
{

private:
	bool toneSafe[20] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	bool toneArmed[20] = { 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	bool toneDirCmd[20] = { 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	bool toneBatteryLow[20] = { 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
	unsigned short toneIndex;
	unsigned short buzzer_pin;
	unsigned short buzzer_pwm_channel;
	unsigned short dutyCycle_In_Bits;
public:

public:
	cBuzzerMelody(unsigned short, unsigned short);
	~cBuzzerMelody();
	void play(unsigned short);
};

