#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#include <Filters.h>

#include "BrushlessMotor.h"
#include "MotionProfile.h"
#include "config.h"

#define TRACKRUN 0
#define TUNING 1

struct ControllerPIDParams{
  volatile float kP, kPI, kPD, kV, kA, kJ;
  volatile float updatePeriod;
};

class Controller {
  private:
    HWCDC* debugSerial;

    volatile int* ticks;
    BrushlessMotor* motor;
    MotionProfile* pathProfile;

    //IMU
    BMI270* imu;

    //control characteristics
    float t_0, t_start;
    int ticksOffset;
    bool isEnabled;
    ControllerPIDParams* PIDParams;

    //hardware characteristics
    float ticksPerMeter;

    //Error accumulator
    float totalPosErr;
    float oldError;

    //Filters
    FilterOnePole* velocityFilter;
    FilterOnePole* accelerationFilter;

    
    
  public:
    //measurements
    volatile float currentPos, currentVel, currentAcc, currentJer, currentHeading;
    volatile float profPosSet, profVelSet, profAccSet, profJerSet;
    volatile float posSetpoint;
    volatile int mode;


    Controller(
      HWCDC* serial,
      volatile int* pTicks, 
      MotionProfile* pPathProfile, BrushlessMotor* pMotor,
      ControllerPIDParams* pPIDParams, BMI270* pImu,
      FilterOnePole* pVelocityFilter, FilterOnePole* pAccelerationFilter,
      float iTicksPerMeter
      );

    void init();
    void enable();
    void disable();
    void start();
    void setMode(int newMode);

    int update();
};

#endif
