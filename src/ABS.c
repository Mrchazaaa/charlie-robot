#include <stdio.h>
#include "ABS.h"

#define OFF 0 //state used to represent the ABS as being non-active

static float TIMERS[4] = {-1, -1, -1, -1};

#define PI 3.14159265358979323846

static int phaseStates[4] = {OFF, OFF, OFF, OFF}; //used to remember which state the ABS algorithm is currently in (one for each wheel) 

static float inputPressure; 
static float *wheelBrakeCMD[4]; 
static float wheelSpinVelocity[4] = {0, 0, 0, 0};
static float wheelSpinAcceleration[4] = {0, 0, 0, 0};
static float wheelSlipAcceleration[4] = {0, 0, 0, 0}; 
static float wheelSlip[4] = {0, 0, 0, 0}; 
static float lastTimeStamp = 0;
static float deltaTime = 0; //Time since last ABS call
static float vehicleSpeed;

float maxWheelVelocity(float wheelVelocity[4]) {
  float maxVelocity = wheelVelocity[0];

  int i;
  for ( i = 1 ; i < 4 ; i++ )
  {
    if (wheelVelocity[i] > maxVelocity) {
      maxVelocity = wheelVelocity[i];
    }
  }

  return maxVelocity;
}

void cycleABS( float newInputPressure, float *brakeCMD[4], float *newWheelSpinVelocity[4], float *newWheelSlipAcceleration[4], float newTimeStamp, float refSpeed )
{
  float rad = 0.3179f;
  float velocityConst = 0.159155 * 2 * PI * rad; 
  //wheel1 spinvel is: 0.159155 * *wheelSpinVelocity[0] * 2 * PI * oCar->_wheelRadius(0)

  if (lastTimeStamp == 0) {
    lastTimeStamp = newTimeStamp;
  }

  wheelBrakeCMD[0] = brakeCMD[0];
  wheelBrakeCMD[1] = brakeCMD[1];
  wheelBrakeCMD[2] = brakeCMD[2];
  wheelBrakeCMD[3] = brakeCMD[3];

  //calculate wheel spin acceleration
  float newDeltaTime = (newTimeStamp - lastTimeStamp);

  deltaTime = newDeltaTime;

  wheelSpinAcceleration[0] = ((*newWheelSpinVelocity[0] * velocityConst)  - wheelSpinVelocity[0])/newDeltaTime;
  wheelSpinAcceleration[1] = ((*newWheelSpinVelocity[1] * velocityConst) - wheelSpinVelocity[1])/newDeltaTime;
  wheelSpinAcceleration[2] = ((*newWheelSpinVelocity[2] * velocityConst) - wheelSpinVelocity[2])/newDeltaTime;
  wheelSpinAcceleration[3] = ((*newWheelSpinVelocity[3] * velocityConst) - wheelSpinVelocity[3])/newDeltaTime;
  
  //update wheel spin velocity
  wheelSpinVelocity[0] = *newWheelSpinVelocity[0] * velocityConst;
  wheelSpinVelocity[1] = *newWheelSpinVelocity[1] * velocityConst;
  wheelSpinVelocity[2] = *newWheelSpinVelocity[2] * velocityConst;
  wheelSpinVelocity[3] = *newWheelSpinVelocity[3] * velocityConst;

  //update wheel slip acceleration
  wheelSlipAcceleration[0] = *newWheelSlipAcceleration[0];
  wheelSlipAcceleration[1] = *newWheelSlipAcceleration[1];
  wheelSlipAcceleration[2] = *newWheelSlipAcceleration[2];
  wheelSlipAcceleration[3] = *newWheelSlipAcceleration[3];

  //calculate vehicle speed
  vehicleSpeed = refSpeed;//maxWheelVelocity(wheelSpinVelocity);//refSpeed;

  wheelSlip[0] = (vehicleSpeed - wheelSpinVelocity[0]) / vehicleSpeed;
  wheelSlip[1] = (vehicleSpeed - wheelSpinVelocity[1]) / vehicleSpeed;
  wheelSlip[2] = (vehicleSpeed - wheelSpinVelocity[2]) / vehicleSpeed;
  wheelSlip[3] = (vehicleSpeed - wheelSpinVelocity[3]) / vehicleSpeed; 
  
  inputPressure = newInputPressure;
  //wheelBrakeCMD = wheelBrakeCMD; 
  lastTimeStamp = newTimeStamp;
 
  //only activate ABS if threshold values are exceeded
  if ( vehicleSpeed > MIN_VEHICLE_VELOCITY_THRESHOLD 
     && inputPressure > MIN_PRESSURE_THRESHOLD/MAX_BRAKE_PRESSURE ) {
    
    //only activate ABS for the wheels whos individual velocity thresholds are exceeded
    int i;
    for ( i = 0 ; i < 4 ; i++ )
    {
      //float wheelSlip = (vehicleSpeed - wheelVelocity[i])/vehicleSpeed ;

      if ( wheelSpinVelocity[i] > MIN_WHEEL_VELOCITY_THRESHOLD ) {
        phase(i);
        //*wheelBrakeCMD[i] = *wheelBrakeCMD[i];
      } else {
        phaseStates[i] = OFF;
      }
    }
  } else { //if thresholds are not met, turn off ABS
    phaseStates[0] = OFF;
    phaseStates[1] = OFF;
    phaseStates[2] = OFF;
    phaseStates[3] = OFF;

    *wheelBrakeCMD[0] = inputPressure;
    *wheelBrakeCMD[1] = inputPressure;
    *wheelBrakeCMD[2] = inputPressure;
    *wheelBrakeCMD[3] = inputPressure;
  }

  return;
}

void phase(int wheel) {
  //printf("state %d wheel %d accel is %.6f\n", phaseStates[wheel], wheel, wheelSpinAcceleration[wheel]);
  
  switch(phaseStates[wheel]) {
    case OFF:
    {
      phaseStates[wheel] = 1;
      break;
    }
    case 1: //Initial application 
      {      
      *wheelBrakeCMD[wheel] = inputPressure;
      
      //if wheel spin acceleration threshold is surpassed, continue to phase 2
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 2;
      }
      break;
      }
    case 2: //Maintain pressure
      {
      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //until the tire longitudinal slip exceeds the slip associated with the Slip Threshold, continue to phase 3
      if (wheelSlip[wheel] > MAX_WHEEL_SLIP) {
        //current tire slip is stored and used as the slip threshold criterion in later phases
        //#define MAX_WHEEL_SLIP wheelSlip[wheel];

        phaseStates[wheel] = 3;
      }

      break;
      }
    case 3: //Reduce pressure 
      {
      //calculate how much pressure should have been released since last ABS call (deltaTime)
      float pressureToRelease = deltaTime * RELEASE_RATE; //* 4; //REMOVE REMOVE REMOVE TESTING TESTING;
      //convert pressure to brake cmd equivalent
      float cmdReleasePressure = pressureToRelease/MAX_BRAKE_PRESSURE;
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel] - cmdReleasePressure;
      
      if (wheelSpinAcceleration[wheel] > MAX_WHEEL_SPIN_ACCELERATION  /*0*/) {
        phaseStates[wheel] = 4;
      }
      break;
      }
    case 4: //Maintain pressure
      {
      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //IMPLEMENT delay
      if (TIMERS[wheel] == -1) { //I.E. the timer has not been set yet
        TIMERS[wheel] = APPLY_DELAY;
      } else {
        TIMERS[wheel] -= deltaTime;
      }

      //if apply delay has elapsed
      if (TIMERS[wheel] < 0 ) {
        TIMERS[wheel] = -1;
        phaseStates[wheel] = 5; 
      }

      if (MAX_WHEEL_SPIN_ACCELERATION * 10 < wheelSpinAcceleration[wheel]) {
        TIMERS[wheel] = -1;
        phaseStates[wheel] = 5;
      }
      //until the wheel spin acceleration (positive) exceeds +A, a
      //multiple (normally 10x) of the Wheel Maximum Spin
      //Acceleration, +a, (signifying the wheel spin velocity is
      //increasing at an excessive rate).

      break;
      }
    case 5: //Increase pressure
      {
      //IMPLEMENT pressure increase

      //calculate how much pressure should have been increased since last ABS call (deltaTime)
      float pressureToApply = deltaTime * PRIMARY_APPLY_RATE;// * 4; //REMOVE REMOVE REMOVE TESTING TESTING;
      //convert pressure to brake cmd equivalent
      float cmdApplyPressure = pressureToApply/MAX_BRAKE_PRESSURE;
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel] + cmdApplyPressure;

      //until the wheel spin acceleration drops and again becomes negative
      if (wheelSpinAcceleration[wheel] < 0 ) {
        phaseStates[wheel] = 6;
      }
      break; 
      }
    case 6: //Maintain pressure
      {
      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //IMPLEMENT delay
      if (TIMERS[wheel] == -1) { //I.E. the timer has not been set yet
        TIMERS[wheel] = APPLY_DELAY;
      } else {
        TIMERS[wheel] -= deltaTime;
      }

      //if apply delay has elapsed
      if (TIMERS[wheel] < 0 ) {
        TIMERS[wheel] = -1;
        phaseStates[wheel] = 7; 
      }

      //until wheel angular acceleration again exceeds the Wheel Minimum Wheel Spin Acceleration (negative)
      //if wheel spin acceleration threshold is surpassed, continue to phase 7
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        TIMERS[wheel] = -1;
        phaseStates[wheel] = 7;
      }

      break;
      }
    case 7: //Increase pressure
      {
      //calculate how much pressure should have been increased since last ABS call (deltaTime)
      float pressureToApply = deltaTime * SECONDARY_APPLY_RATE;// * 4; //REMOVE REMOVE REMOVE TESTING TESTING
      //convert pressure to brake cmd equivalent
      float cmdApplyPressure = pressureToApply/MAX_BRAKE_PRESSURE;

      if (*wheelBrakeCMD[wheel] >= 1.0f) {
        *wheelBrakeCMD[wheel] = 1.0f;
      } else {
        *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel] + cmdApplyPressure;
      }

      //until wheel angular acceleration drops below the Wheel Minimum Angular Acceleration (negative)
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 3;//8;
      }
    
      break; 
      }
    //case 8: //Reduce pressure (a cycle is complete, return to phase 3)
    //  {
    //  phaseStates[wheel] = 3;
    //  break;
  
    default : 
      {
        printf("ABS state error\n");
        break;
      }
  }
}