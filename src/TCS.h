 #ifndef TCS_H_
 #define TCS_H_

#define FL 0 //used to access wheel index for FRONT LEFT
#define FR 1 //used to access wheel index for FRONT RIGHT
#define RL 2 //used to access wheel index for REAR LEFT
#define RR 3 //used to access wheel index for REAR RIGHT

//threshold values above which ABS is activated
#define MAX_WHEEL_SLIP                 0.15

 int getPhaseStates(int index) ;
 float getWheelSpinVelocity(int index) ;
 float getWheelSpinAcceleration(int index) ;
 float getWheelSlipAcceleration(int index) ;
 float getWheelSlip(int index) ;
 float getDeltaTime();
 void phase(int wheel) ;
 float getVehicleSpeed() ;

 void cycleTCS( float newInputAccel, float *accelCmd, float *newWheelSpinVelocity[4], float newTimeStamp );

 #endif