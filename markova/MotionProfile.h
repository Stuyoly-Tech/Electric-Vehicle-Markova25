#ifndef MOTION_PROFILE_H__
#define MOTION_PROFILE_H__

#define MOTION_PROFILE_TABLE_SIZE 1024

#define MOTION_PROFILE_SUCCESS 1
#define MOTION_PROFILE_VEL_LIM 2
#define MOTION_PROFILE_ACC_LIM 3
#define MOTION_PROFILE_INT_LIM 4

#include <Arduino.h>
#include <HardwareSerial.h>

struct MotionProfileParameters {
  volatile float dist, vMax, aMax, jMax;
  volatile float secondsPerInterval, initialDelay;
};

class MotionProfile {
  private:
    HWCDC* debugSerial;

    float xTable[MOTION_PROFILE_TABLE_SIZE];
    float vTable[MOTION_PROFILE_TABLE_SIZE];
    float aTable[MOTION_PROFILE_TABLE_SIZE];
    float jTable[MOTION_PROFILE_TABLE_SIZE];

  public:
    int numIntervals;
    float pathDuration;
    MotionProfileParameters* params;

    MotionProfile(HWCDC* serial, MotionProfileParameters* pParams);
    int generateMotionProfiles(float t);
    float getPositionProfile(float t);
    float getVelocityProfile(float t);
    float getAccelerationProfile(float t);
    float getJerkProfile(float t);
    
    void clearMotionProfile();

};

#endif