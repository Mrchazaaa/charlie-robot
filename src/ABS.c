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

#define PI 3.14159265358979323846

static int phaseStates[4] = {OFF, OFF, OFF, OFF}; //used to remember which state the ABS algorithm is currently in (one for each wheel) 

static float inputPressure; 
static float *wheelBrakeCMD[4]; 

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

void cycleABS( float inputPressure, float *wheelBrakeCMD[4], float *wheelSpinVelocity[4] )
{

  inputPressure = inputPressure;
  wheelBrakeCMD = wheelBrakeCMD;

  //calculate wheel speed
  float wheelVelocity[4];

  //wheel1 spinvel is: 0.159155 * *wheelSpinVelocity[0] * 2 * PI * oCar->_wheelRadius(0) 
  wheelVelocity[0] =   0.159155 * *wheelSpinVelocity[0] * 2 * PI *  0.3179;
  wheelVelocity[1] =   0.159155 * *wheelSpinVelocity[0] * 2 * PI *  0.3179;
  wheelVelocity[2] =   0.159155 * *wheelSpinVelocity[0] * 2 * PI *  0.3179;
  wheelVelocity[3] =   0.159155 * *wheelSpinVelocity[0] * 2 * PI *  0.3179;

  //calculate vehicle speed
  float vehicleSpeed = maxWheelVelocity(wheelVelocity);
  
  //only activate ABS if threshold values are exceeded
  if ( vehicleSpeed > MIN_VEHICLE_VELOCITY_THRESHOLD 
     && inputPressure > MIN_PRESSURE_THRESHOLD ) {
    
    //only activate ABS for the wheels whos individual velocity thresholds are exceeded
    int i;
    for ( i = 0 ; i < 4 ; i++ )
    {
      //float wheelSlip = (vehicleSpeed - wheelVelocity[i])/vehicleSpeed ;

      if ( wheelVelocity[i] > MIN_WHEEL_VELOCITY_THRESHOLD ) {
        phase(i);
        return;
      }
    }

  } 
  //if thresholds are not met, turn off ABS

  phaseStates[0] = OFF;
  phaseStates[1] = OFF;
  phaseStates[2] = OFF;
  phaseStates[3] = OFF;

  //*wheelBrakeCMD[FL] = 1.0f;

  return;
}

void phase(int wheel) {
  printf("wheel %d state is %d\n", wheel, phaseStates[wheel]);
  
  switch(phaseStates[wheel]) {
    case 0:
      phaseStates[wheel] = 1;
      break;

    case 1: 
      
      *wheelBrakeCMD[wheel] = inputPressure;
      

      //phaseStates[wheel] = 2;

      break;
	
    case 2:
      phaseStates[wheel] = 3;
      break;

    case 3:
      phaseStates[wheel] = 4;
      break;

    case 4:
      phaseStates[wheel] = 5;
      break;

    case 5:
      phaseStates[wheel] = 6;
      break; 
	
    case 6:
      phaseStates[wheel] = 7;
      break;

    case 7:
      phaseStates[wheel] = 8;
      break; 

    case 8:
      phaseStates[wheel] = 3;
      break;
  
    default : 
      printf("ABS state error\n");
      break;
  }
}