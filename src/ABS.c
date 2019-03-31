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
//#define MIN_PRESSURE_THRESHOLD         NULL

static int phaseStates[4] = {OFF, OFF, OFF, OFF}; //used to remember which state the ABS algorithm is currently in (one for each wheel) 

void cycleABS( float inputPressure, float *wheelBrakeCMD[4], float *wheelSpinVelocity[4] )
{

  //calculate wheel speed
  float wheelVelocity[4];


  //calculate vehicle speed
  float vehicleSpeed;
  
  //only activate ABS if threshold values are exceeded
  if ( vehicleSpeed > MIN_VEHICLE_VELOCITY_THRESHOLD 
     && inputPressure > MIN_PRESSURE_THRESHOLD ) {
    
    //phase();

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

    case 1:
      phaseStates[wheel] = 2;
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