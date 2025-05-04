#ifndef UTILS_H
#define UTILS_H

int meters_to_ticks(float d, float tpm);
float ticks_to_meters(int t, float tpm);
bool get_btn_state(int index, int *btn_pins, bool* btn_states);

#endif 
