#include <stdio.h>
#include "TCS.h"

static float TIMERS[4] = {-1, -1, -1, -1};

#define PI 3.14159265358979323846

static float inputAccel; 
static float wheelSpinVelocity[4] = {0, 0, 0, 0};
static float wheelSlip[4] = {0, 0, 0, 0}; 
static float lastTimeStamp = 0;
static float deltaTime = 0; //Time since last TCS call
static float vehicleSpeed;

void cycleTCS( float newInputAccel, float *accelCmd, float *newWheelSpinVelocity[4], float newTimeStamp )
{
  float rad = 0.3179f;
  float velocityConst = 0.159155 * 2 * PI * rad; 
  //wheel1 spinvel is: 0.159155 * *wheelSpinVelocity[0] * 2 * PI * oCar->_wheelRadius(0)

  if (lastTimeStamp == 0) {
    lastTimeStamp = newTimeStamp;
  }

  //calculate wheel spin acceleration
  float newDeltaTime = (newTimeStamp - lastTimeStamp);

  deltaTime = newDeltaTime;
  
  //update wheel spin velocity
  wheelSpinVelocity[0] = *newWheelSpinVelocity[0] * velocityConst;
  wheelSpinVelocity[1] = *newWheelSpinVelocity[1] * velocityConst;
  wheelSpinVelocity[2] = *newWheelSpinVelocity[2] * velocityConst;
  wheelSpinVelocity[3] = *newWheelSpinVelocity[3] * velocityConst;

  //calculate vehicle speed (acceptable to just use max speed of undriven wheels)
  vehicleSpeed = (wheelSpinVelocity[FR] > wheelSpinVelocity[FL]) ? wheelSpinVelocity[FR]: wheelSpinVelocity[FL];

  wheelSlip[0] = (vehicleSpeed - wheelSpinVelocity[0]) / vehicleSpeed;
  wheelSlip[1] = (vehicleSpeed - wheelSpinVelocity[1]) / vehicleSpeed;
  wheelSlip[2] = (vehicleSpeed - wheelSpinVelocity[2]) / vehicleSpeed;
  wheelSlip[3] = (vehicleSpeed - wheelSpinVelocity[3]) / vehicleSpeed; 
   
  lastTimeStamp = newTimeStamp;
 
  //only activate TCS if threshold values are exceeded
   if ( *accelCmd > 0.0 ) {
     //regulate engine
     //regulate differential
   } else { //if thresholds are not met, turn off TCS
     *accelCmd = newInputAccel;
   }

  return;
}
