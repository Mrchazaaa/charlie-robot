#include <stdio.h>

#define FL 0; //used to access wheel index for FRONT LEFT
#define FR 1; //used to access wheel index for FRONT RIGHT
#define RL 2; //used to access wheel index for REAR LEFT
#define RR 3; //used to access wheel index for REAR RIGHT

static int phaseState = 1; //used to remember which state the ABS algorithm is currently in 


void cycleABS( float *wheelBrakeCMD[4], float *wheelSpinVelocity[4] )
{


  
  //phase();
  
  *wheelBrakeCMD[0] = 1.0f;

  return;
}

void phase() {
  printf("state is %d\n", phaseState);
  
  switch(phaseState) {

    case 1:
      phaseState = 2;
      break;
	
    case 2:
      phaseState = 3;
      break;

    case 3:
      phaseState = 4;
      break;

    case 4:
      phaseState = 5;
      break;

    case 5:
      phaseState = 6;
      break; 
	
    case 6:
      phaseState = 7;
      break;

    case 7:
      phaseState = 8;
      break; 

    case 8:
      phaseState = 3;
      break;
  
    default : 
      printf("ABS state error\n");
      break;
  }
}