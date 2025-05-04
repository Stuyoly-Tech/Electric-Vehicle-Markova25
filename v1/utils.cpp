#include <Arduino.h>
#include "utils.h"

int meters_to_ticks(float d, float tpm) {
	return d*tpm;
}

float ticks_to_meters(int t,float tpm) {
	return (float)t/tpm;
}

bool get_btn_state(int index, int *btn_pins, bool* btn_states) {
	bool buttonstate = digitalRead(btn_pins[index]);
  	if (buttonstate != btn_states[index]) {
		btn_states[index] = buttonstate;
    	if (buttonstate == HIGH) {
      		return true;
    	}	
  }
  return false;
}
