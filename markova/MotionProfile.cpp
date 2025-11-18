#include "MotionProfile.h"
#include "config.h"

MotionProfile::MotionProfile(HWCDC* serial, MotionProfileParameters* pParams) {
  debugSerial = serial;
  params = pParams;
  pathDuration = 0;
  numIntervals = 0;
  clearMotionProfile();
}

int MotionProfile::generateMotionProfiles(float t) {
  int return_val = 1;
  float secondsPerInterval = params->secondsPerInterval;
  float initialDelay = params->initialDelay;
  pathDuration += initialDelay;

  float d = params->dist;
  float v = params->vMax;
  float a = params->aMax;
  float j = params->jMax;

  v = (a*t - a*sqrt(sq(t)-4*d/a))/2;


  // If the target velocity is too high
  if ((pow(v, 2) * j + v * pow(a, 2)) / (a * j) > d) {
    float s1 = (-v + sqrt(pow(v, 2) + 4 * d * pow(j, 2) * a)) / (2 * j);
    float s2 = (-v - sqrt(pow(v, 2) + 4 * d * pow(j, 2) * a)) / (2 * j);
    v = (s1 > s2) ? s1 : s2;
    // For safety
    v -= 0.0001;
    return_val = MOTION_PROFILE_VEL_LIM;
  }

  // If acceleration is too high
  if (a > sqrt(v * j)) {
    a = sqrt(v * j) - 0.0001;
    return_val = MOTION_PROFILE_ACC_LIM;
  }

  // Duration of transitionary period
  float tCurve = (v * j + pow(a, 2)) / (a * j);
  float tCoast = (d - v * tCurve) / v;
  pathDuration = 2 * tCurve + tCoast;

  // Calculate amount of indices
  numIntervals = (int)(pathDuration / secondsPerInterval) + 1;
  if (numIntervals > MOTION_PROFILE_TABLE_SIZE-1) {
    numIntervals = MOTION_PROFILE_TABLE_SIZE-1;
    secondsPerInterval = pathDuration/numIntervals;
    return_val = MOTION_PROFILE_INT_LIM;
  }

  // Calculate time intervals
  float t_interval[7] = {0};
  t_interval[0] = a / j;
  t_interval[1] = tCurve - t_interval[0];
  t_interval[2] = tCurve;
  t_interval[3] = tCurve + tCoast;
  t_interval[4] = t_interval[3] + t_interval[0];
  t_interval[5] = pathDuration - t_interval[0];
  t_interval[6] = pathDuration;

  /*
  for (int i=0; i<7; i++) {
    debugSerial->println(t_interval[i]);
  }
  */

  // Useful buffers
  float xbuff[7] = {0};
  float vbuff[7] = {0};

  vbuff[0] = (j / 2) * pow(t_interval[0], 2);
  vbuff[1] = a * (t_interval[1] - t_interval[0]) + vbuff[0];
  vbuff[2] = -(j / 2) * pow(t_interval[2] - t_interval[1], 2) + a * (t_interval[2] - t_interval[1]) + vbuff[1];
  vbuff[3] = v;
  vbuff[4] = -(j / 2) * pow(t_interval[4] - t_interval[3], 2) + v;
  vbuff[5] = -a * (t_interval[5] - t_interval[4]) + vbuff[4];
  vbuff[6] = 0;

  xbuff[0] = (j / 6) * pow(t_interval[0], 3);
  xbuff[1] = (a / 2) * pow(t_interval[1] - t_interval[0], 2) + vbuff[0] * (t_interval[1] - t_interval[0]) + xbuff[0];
  xbuff[2] = -(j / 6) * pow(t_interval[2] - t_interval[1], 3) + (a / 2) * pow(t_interval[2] - t_interval[1], 2) + vbuff[1] * (t_interval[2] - t_interval[1]) + xbuff[1];
  xbuff[3] = v * (t_interval[3] - t_interval[2]) + xbuff[2];
  xbuff[4] = -(j / 6) * pow(t_interval[4] - t_interval[3], 3) + v * (t_interval[4] - t_interval[3]) + xbuff[3];
  xbuff[5] = -(a / 2) * pow(t_interval[5] - t_interval[4], 2) + vbuff[4] * (t_interval[5] - t_interval[4]) + xbuff[4];
  xbuff[6] = d;

  // Generate motion profile points
  for (int i = 0; i < numIntervals; i++) {
    float t = i * secondsPerInterval;
    if (t < t_interval[0]) {
      xTable[i] = (j / 6) * pow(t, 3);
      vTable[i] = (j / 2) * pow(t, 2);
      aTable[i] = j * t;
      jTable[i] = j;

      //xbuff[0] = xTable[i];
      //vbuff[0] = vTable[i];
    } 
    else if (t <= t_interval[1]) {
      xTable[i] = (a / 2) * pow(t - t_interval[0], 2) + vbuff[0] * (t - t_interval[0]) + xbuff[0];
      vTable[i] = a * (t - t_interval[0]) + vbuff[0];
      aTable[i] = a;
      jTable[i] = 0;
      
      //xbuff[1] = xTable[i];
      //vbuff[1] = vTable[i];
    } 
    else if (t <= t_interval[2]) {
      xTable[i] = -(j / 6) * pow(t - t_interval[1], 3) + (a / 2) * pow(t - t_interval[1], 2) + vbuff[1] * (t - t_interval[1]) + xbuff[1];
      vTable[i] = -(j / 2) * pow(t - t_interval[1], 2) + a * (t - t_interval[1]) + vbuff[1];
      aTable[i] = -j * (t - t_interval[1]) + a;
      jTable[i] = -j;

      //xbuff[2] = xTable[i];
      //vbuff[2] = vTable[i];
    } 
    else if (t <= t_interval[3]) {
      xTable[i] = v * (t - t_interval[2]) + xbuff[2];
      vTable[i] = v;
      aTable[i] = 0;
      jTable[i] = 0;

      //xbuff[3] = xTable[i];
      //vbuff[3] = vTable[i];
    } 
    else if (t <= t_interval[4]) {
      xTable[i] = -(j / 6) * pow(t - t_interval[3], 3) + v * (t - t_interval[3]) + xbuff[3];
      vTable[i] = -(j / 2) * pow(t - t_interval[3], 2) + v;
      aTable[i] = -j * (t - t_interval[3]);
      jTable[i] = -j;

      //xbuff[4] = xTable[i];
      //vbuff[4] = vTable[i];
    } 
    else if (t <= t_interval[5]) {
      xTable[i] = -(a / 2) * pow(t - t_interval[4], 2) + vbuff[4] * (t - t_interval[4]) + xbuff[4];
      vTable[i] = -a * (t - t_interval[4]) + vbuff[4];
      aTable[i] = -a;
      jTable[i] = 0;

      //xbuff[5] = xTable[i];
      //vbuff[5] = vTable[i];
    } 
    else if (t <= t_interval[6]) {
      xTable[i] = (j / 6) * pow(t - t_interval[5], 3) - (a / 2) * pow(t - t_interval[5], 2) + vbuff[5] * (t - t_interval[5]) + xbuff[5];
      vTable[i] = (j / 2) * pow(t - t_interval[5], 2) - a * (t - t_interval[5]) + vbuff[5];
      aTable[i] = j * (t - t_interval[5]) - a;
      jTable[i] = j;

      //xbuff[6] = xTable[i];
      //vbuff[6] = vTable[i];
    } 
    else {
      xTable[i] = xbuff[6];
      vTable[i] = 0;
      aTable[i] = 0;
      jTable[i] = 0;
    }
  }

  return return_val;
}

float MotionProfile::getPositionProfile(float t) {
  if (t < params->initialDelay) {
    return 0;
  }
  t -= params->initialDelay;
  int index = (int)(t / params->secondsPerInterval);
  // Outside time interval
  if (index >= numIntervals - 1) {
    return xTable[numIntervals - 1];
  }
  // Interpolate points
  float slope = (xTable[index + 1] - xTable[index]) / (params->secondsPerInterval);
  return (t - index * params->secondsPerInterval) * slope + xTable[index];
}

float MotionProfile::getVelocityProfile(float t) {
  if ((t < params->initialDelay) || (t > pathDuration)) {
    return 0;
  }
  t -= params->initialDelay;
  int index = (int)(t / params->secondsPerInterval);
  if (index == numIntervals - 1) {
    return vTable[index];
  }
  // Interpolate points
  float slope = (vTable[index + 1] - vTable[index]) / (params->secondsPerInterval);
  return (t - index * params->secondsPerInterval) * slope + vTable[index];
}

float MotionProfile::getAccelerationProfile(float t) {
  if ((t < params->initialDelay) || (t > pathDuration)) {
    return 0;
  }
  t -= params->initialDelay;
  int index = (int)(t / params->secondsPerInterval);
  if (index == numIntervals - 1) {
    return aTable[index];
  }
  // Interpolate points
  float slope = (aTable[index + 1] - aTable[index]) / (params->secondsPerInterval);
  return (t - index * params->secondsPerInterval) * slope + aTable[index];
}

float MotionProfile::getJerkProfile(float t) {
  if ((t < params->initialDelay) || (t > pathDuration)) {
    return 0;
  }
  t -= params->initialDelay;
  int index = (int)(t / params->secondsPerInterval);
  return jTable[index];
}

void MotionProfile::clearMotionProfile() {
  numIntervals = 0;
  pathDuration = 0;
  //Zero out tables
  for (int i=0; i<MOTION_PROFILE_TABLE_SIZE; i++) {
    xTable[i] = 0;
    vTable[i] = 0;
    aTable[i] = 0;
    jTable[i] = 0;
  }
}