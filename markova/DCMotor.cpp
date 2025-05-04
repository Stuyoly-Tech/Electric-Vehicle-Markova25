#include <Arduino.h>
#include "DCMotor.h"

DCMotor::DCMotor
(
 uint8_t iOut, uint8_t iDir, uint8_t iISense, uint8_t iNSleep,
 float iISenseR, uint32_t iFreq
) 
{
	out = iOut;
	dir = iDir;
	isense = iISense;
	isenseR = iISenseR;
    nsleep = iNSleep;
	freq = iFreq;
	
	pinMode(out, OUTPUT);
	pinMode(dir, OUTPUT);
	pinMode(isense, INPUT);
    pinMode(nsleep, OUTPUT);
    digitalWrite(nsleep, LOW);

	ledcAttach(out, freq, 10);
}

void DCMotor::set_power(float p) {
	//Set direction
	digitalWrite(dir, (p >= 0));
	//Clamp p
	if (abs(p) > 1) {
		p /= p;
	}
	uint32_t duty = (uint32_t)(p * (pow(2, 10)-1));
	ledcWrite(out, duty);
}

void DCMotor::enable() {
  digitalWrite(nsleep, HIGH);
}

void DCMotor::disable() {
  digitalWrite(nsleep, LOW);
}

float DCMotor::get_current_draw() {
	return (analogRead(isense)/(pow(2, 12)) * 3.3)/isenseR;
}



