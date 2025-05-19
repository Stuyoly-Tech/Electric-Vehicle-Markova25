#include <Arduino.h>
#include "BrushlessMotor.h"

BrushlessMotor::BrushlessMotor(
  uint8_t iOut,
  uint32_t iFull_reverse_pulse,
  uint32_t iNeutral_min_pulse,
  uint32_t iNeutral_max_pulse,
  uint32_t iFull_forward_pulse,
  uint32_t iFreq,
  uint32_t iPeriod) {
	out = iOut;
	full_reverse_pulse = iFull_reverse_pulse;
	neutral_min_pulse = iNeutral_min_pulse;
  neutral_max_pulse = iNeutral_max_pulse;
	full_forward_pulse = iFull_forward_pulse;
	freq = iFreq;
	period = iPeriod;

	center = (full_reverse_pulse + full_forward_pulse) / 2;
	neutral_radius = center - neutral_min_pulse;

	pinMode(out, OUTPUT);
	ledcAttach(out, freq, 10);
}

void BrushlessMotor::set_power(float p) {

	if (abs(p) > 1) {
		p /= p;
	}


	uint32_t duty = (uint32_t)((get_pulse_width(p) / period) * (pow(2, 10) - 1));

	ledcWrite(out, duty);
}

float BrushlessMotor::get_pulse_width(float p) {
	if(p == 0){
		return (float)center;
	}

	float n;

	n = abs(p * (full_forward_pulse - neutral_max_pulse));

	if(p<0){
		return center - neutral_radius + n;
	}
	if(p>0){
		return center + neutral_radius - n;
	}

}

void BrushlessMotor::disable(){
	set_power(0);
}