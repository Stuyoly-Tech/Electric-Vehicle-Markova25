#ifndef DCMOTOR_H
#define DCMOTOR_H

class DCMotor {
	private:
		//Motor pins
		uint8_t out, dir, isense, nsleep;
		uint32_t freq;
		float isenseR;
	public:
		DCMotor(
			uint8_t iOut, uint8_t iDir, uint8_t iISense, uint8_t iNSleep,
			float iISenseR, uint32_t iFreq
			);
    void enable();
	void set_power(float p);
    void disable();
	float get_current_draw();
};


#endif
