#include "Controller.h"
#include "utils.h"

Controller::Controller(
  HWCDC* serial,
  volatile int* pTicks, 
  MotionProfile* pPathProfile, BrushlessMotor* pMotor,
  ControllerPIDParams* pPIDParams, BMI270* pImu,
  FilterOnePole* pVelocityFilter, FilterOnePole* pAccelerationFilter,
  float iTicksPerMeter
) 
{
  debugSerial = serial;
  ticks = pTicks;
  pathProfile = pPathProfile;
  motor = pMotor;
  PIDParams = pPIDParams;
  ticksPerMeter = iTicksPerMeter;
  imu = pImu;
  velocityFilter = pVelocityFilter;
  accelerationFilter = pAccelerationFilter;
  posSetpoint = 0;

  isEnabled = false;
  mode = TRACKRUN;
}

void Controller::init() {
  currentPos = 0;
  currentVel = 0;
  currentAcc = 0;
  currentJer = 0;
  currentHeading = 0;
  profPosSet = 0;
  profVelSet = 0;
  profAccSet = 0;
  profJerSet = 0;
  ticksOffset = 0;
  totalPosErr = 0;
  oldError = 0;
  t_0 = micros();
  t_start = micros();
}

void Controller::enable() {
  isEnabled = true;
}

void Controller::disable() {
  isEnabled = false;
}

void Controller::start() {
  init();
  ticksOffset = *ticks;
}

int Controller::update() {
  float t_now = micros();
  float t = (float)(t_now - t_start)/pow(10, 6);
  float delta_t = (float)(t_now - t_0)/pow(10, 6);

  if (delta_t > PIDParams->updatePeriod) {
    //Calculate kinematic values
    float newPos = ticks_to_meters(*ticks - ticksOffset, ticksPerMeter);
    float newVel = (newPos - currentPos)/delta_t;
    velocityFilter->input(newVel);
    float newAcc = (velocityFilter->output() - currentVel)/delta_t;
    accelerationFilter->input(newAcc);

    //PID
    if (isEnabled) {
      float error;
      if (mode == TRACKRUN) {
        profPosSet = pathProfile->getPositionProfile(t);
        profVelSet = pathProfile->getVelocityProfile(t);
        profAccSet = pathProfile->getAccelerationProfile(t);

        //For PID
        error = profPosSet - newPos;
        float dtError = (error-oldError)/delta_t;

        totalPosErr += error*delta_t;
        motor->set_power(
          error*PIDParams->kP + totalPosErr*PIDParams->kPI + dtError*PIDParams->kPD
        );
      }
      else if (mode == TUNING) {
        error = posSetpoint-newPos;
        float dtError = (error-oldError)/delta_t;
        totalPosErr += error*delta_t;
        motor->set_power(error*PIDParams->kP+ totalPosErr*PIDParams->kPI + dtError*PIDParams->kPD);
      }
      oldError = error;
    }
    
    t_0 = t_now;

    //Update values
    currentPos = newPos;
    currentVel = velocityFilter->output();
    currentAcc = accelerationFilter->output();
  }

  if (mode == TUNING) {
    return 1;
  }
  
  if (t >= pathProfile->pathDuration) {
    return (abs(currentPos - profPosSet) > STOP_RANGE);
  }

  return isEnabled;
}

void Controller::setMode(int newMode) {
  mode = newMode;
}