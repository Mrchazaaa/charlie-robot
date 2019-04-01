#include <stdio.h>

#define OFF 0 //state used to represent the ABS as being non-active

#define FL 0 //used to access wheel index for FRONT LEFT
#define FR 1 //used to access wheel index for FRONT RIGHT
#define RL 2 //used to access wheel index for REAR LEFT
#define RR 3 //used to access wheel index for REAR RIGHT

//threshold values above which ABS is activated
#define MIN_PRESSURE_THRESHOLD         0
#define MIN_VEHICLE_VELOCITY_THRESHOLD 0
#define MIN_WHEEL_VELOCITY_THRESHOLD   0
#define MIN_WHEEL_SPIN_ACCELERATION    0
#define MAX_WHEEL_SPIN_ACCELERATION    0
#define MAX_WHEEL_SLIP                 0

#define PI 3.14159265358979323846

static int phaseStates[4] = {OFF, OFF, OFF, OFF}; //used to remember which state the ABS algorithm is currently in (one for each wheel) 

static float inputPressure; 
static float *wheelBrakeCMD[4]; 
static float wheelSpinVelocity[4] = {0, 0, 0, 0};
static float wheelSpinAcceleration[4] = {0, 0, 0, 0};
static float wheelSlipAcceleration[4] = {0, 0, 0, 0}; 
static float wheelSlip[4] = {0, 0, 0, 0}; 
static float lastTimeStamp = 0;

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

void cycleABS( float newInputPressure, float *wheelBrakeCMD[4], float *newWheelSpinVelocity[4], float *newWheelSlipAcceleration[4], float newTimeStamp )
{

  //calculate wheel spin acceleration
  float deltaTime = newTimeStamp - lastTimeStamp;
  wheelSpinAcceleration[0] = (*newWheelSpinVelocity[0] - wheelSpinVelocity[0])/deltaTime;
  wheelSpinAcceleration[1] = (*newWheelSpinVelocity[1] - wheelSpinVelocity[1])/deltaTime;
  wheelSpinAcceleration[2] = (*newWheelSpinVelocity[2] - wheelSpinVelocity[2])/deltaTime;
  wheelSpinAcceleration[3] = (*newWheelSpinVelocity[3] - wheelSpinVelocity[3])/deltaTime;
  
  //update wheel spin velocity
  wheelSpinVelocity[0] = *newWheelSpinVelocity[0];
  wheelSpinVelocity[1] = *newWheelSpinVelocity[1];
  wheelSpinVelocity[2] = *newWheelSpinVelocity[2];
  wheelSpinVelocity[3] = *newWheelSpinVelocity[3];

  //update wheel slip acceleration
  wheelSlipAcceleration[0] = *newWheelSlipAcceleration[0];
  wheelSlipAcceleration[1] = *newWheelSlipAcceleration[1];
  wheelSlipAcceleration[2] = *newWheelSlipAcceleration[2];
  wheelSlipAcceleration[3] = *newWheelSlipAcceleration[3];

  //calculate vehicle speed
  float vehicleSpeed = maxWheelVelocity(wheelSpinVelocity);

  wheelSlip[0] = (wheelSpinVelocity[0] - vehicleSpeed) / vehicleSpeed;
  wheelSlip[1] = (wheelSpinVelocity[1] - vehicleSpeed) / vehicleSpeed;
  wheelSlip[2] = (wheelSpinVelocity[2] - vehicleSpeed) / vehicleSpeed;
  wheelSlip[3] = (wheelSpinVelocity[3] - vehicleSpeed) / vehicleSpeed; 
  
  inputPressure = newInputPressure;
  wheelBrakeCMD = wheelBrakeCMD; 
  lastTimeStamp = newTimeStamp;
    
  //wheel1 spinvel is: 0.159155 * *wheelSpinVelocity[0] * 2 * PI * oCar->_wheelRadius(0)

  //only activate ABS if threshold values are exceeded
  if ( vehicleSpeed > MIN_VEHICLE_VELOCITY_THRESHOLD 
     && inputPressure > MIN_PRESSURE_THRESHOLD ) {
    
    //only activate ABS for the wheels whos individual velocity thresholds are exceeded
    int i;
    for ( i = 0 ; i < 4 ; i++ )
    {
      //float wheelSlip = (vehicleSpeed - wheelVelocity[i])/vehicleSpeed ;

      if ( wheelSpinVelocity[i] > MIN_WHEEL_VELOCITY_THRESHOLD ) {
        phase(i);
      } else {
        phaseStates[i] = OFF;
      }
    }
  } else { //if thresholds are not met, turn off ABS
    phaseStates[0] = OFF;
    phaseStates[1] = OFF;
    phaseStates[2] = OFF;
    phaseStates[3] = OFF;
  }
  
  //*wheelBrakeCMD[FL] = 1.0f;

  return;
}

void phase(int wheel) {
  printf("wheel %d state is %d\n", wheel, phaseStates[wheel]);
  
  switch(phaseStates[wheel]) {
    case 0:
      phaseStates[wheel] = 1;
      break;

    case 1: //Initial application 
      
      *wheelBrakeCMD[wheel] = inputPressure;
      
      //if wheel spin acceleration threshold is surpassed, continue to phase 2
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 2;
      }
      break;
	
    case 2: //Maintain pressure

      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //until the tire longitudinal slip exceeds the slip associated with the Slip Threshold, continue to phase 3
      if (wheelSlip[wheel] > MAX_WHEEL_SLIP) {
        //current tire slip is stored and used as the slip threshold criterion in later phases
        //#define MAX_WHEEL_SLIP wheelSlip[wheel];

        phaseStates[wheel] = 3;
      }

      break;

    case 3: //Reduce pressure 
      //IMPLEMENT delay
      //phaseStates[wheel] = 4;
      break;

    case 4: //Maintain pressure

      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //IMPLEMENT delay

      if (MAX_WHEEL_SPIN_ACCELERATION * 10 > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 5;
      }
      //until the wheel spin acceleration (positive) exceeds +A, a
      //multiple (normally 10x) of the Wheel Maximum Spin
      //Acceleration, +a, (signifying the wheel spin velocity is
      //increasing at an excessive rate).

      break;

    case 5: //Increase pressure
      //IMPLEMENT pressure increase

      //until the wheel spin acceleration drops and again becomes negative
      if (wheelSpinAcceleration[wheel] < 0 ) {
        phaseStates[wheel] = 6;
      }
      break; 
	
    case 6: //Maintain pressure

      //Output pressure is set equal to previous pressure 
      *wheelBrakeCMD[wheel] = *wheelBrakeCMD[wheel];

      //IMPLEMENT delay

      //until wheel angular acceleration again exceeds the Wheel Minimum Wheel Spin Acceleration (negative)
      //if wheel spin acceleration threshold is surpassed, continue to phase 7
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 7;
      }

      break;

    case 7: //Increase pressure

      //until wheel angular acceleration drops below the Wheel Minimum Angular Acceleration (negative)
      if (MIN_WHEEL_SPIN_ACCELERATION > wheelSpinAcceleration[wheel]) {
        phaseStates[wheel] = 8;
      }
    
      break; 

    case 8: //Reduce pressure (a cycle is complete, return to phase 3)
      phaseStates[wheel] = 3;
      break;
  
    default : 
      printf("ABS state error\n");
      break;
  }
}