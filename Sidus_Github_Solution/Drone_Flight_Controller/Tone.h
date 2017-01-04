// Tone.h

class tone_class
{
private:
	bool toneArmed[20] = { 1, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
	bool toneBatteryLow[20] = { 1, 0, 1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0 };
	unsigned short toneIndex = 0;
	unsigned short toneArmedDutyCycle = 25;
public:
	void playTone(unsigned int);
}buzzer;