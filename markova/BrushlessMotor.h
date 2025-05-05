#ifndef BRUSHLESSMOTOR_H
#define BRUSHLESSMOTOR_H

class BrushlessMotor {
private:
  uint8_t out;
  uint32_t full_reverse_pulse;
  uint32_t neutral_min_pulse;
  uint32_t neutral_max_pulse;
  uint32_t full_forward_pulse;
  uint32_t freq;
  uint32_t period;
  uint32_t center;
  uint32_t neutral_radius;
public:
  BrushlessMotor(
    uint8_t iOut,
    uint32_t iFull_reverse_pulse,
    uint32_t iNeutral_min_pulse,
    uint32_t iNeutral_max_pulse,
    uint32_t iFull_forward_pulse,
    uint32_t iFreq,
    uint32_t iPeriod);
  void set_power(float p);
  float get_pulse_width(float p);
  void disable();
};


#endif
