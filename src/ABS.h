#include <stdio.h>

#ifndef ABS_H_
#define ABS_H_

#define PI 3.14159265358979323846

#define FL 0 //used to index for FRONT LEFT wheel
#define FR 1 //used to index for FRONT RIGHT wheel
#define RL 2 //used to index for REAR LEFT wheel
#define RR 3 //used to index for REAR RIGHT wheel

//threshold values above which ABS is activated
#define MIN_VEHICLE_VELOCITY_THRESHOLD 10       //Met/Sec
#define MIN_WHEEL_VELOCITY_THRESHOLD   10       //Rads/Sec
#define MIN_PRESSURE_THRESHOLD         13000    //Pa
#define APPLY_DELAY                    0.04     //Sec
#define PRIMARY_APPLY_RATE             11000000 //Pa/Sec
#define SECONDARY_APPLY_RATE           8458000  //Pa/Sec
#define RELEASE_RATE                   50000000 //Pa/Sec
#define MIN_WHEEL_SPIN_ACCELERATION    -95      //Rad/Sec^2
#define MAX_WHEEL_SPIN_ACCELERATION    0        //Met/Sec^2
#define INITIAL_MAX_WHEEL_SLIP         0.12     //norm wheel slip
static float MAX_WHEEL_SLIP[] = {INITIAL_MAX_WHEEL_SLIP,
                                 INITIAL_MAX_WHEEL_SLIP, 
                                 INITIAL_MAX_WHEEL_SLIP, 
                                 INITIAL_MAX_WHEEL_SLIP};
#define MAX_BRAKE_PRESSURE             13000000 //Pa
#define WHEEL_RADIUS_STATIC            0.3179f  //Met

 int getPhaseStates(int index) ;
 float getWheelSpinVelocity(int index) ;
 float getWheelSpinAcceleration(int index) ;
 float getWheelSlipAcceleration(int index) ;
 float getWheelSlip(int index) ;
 float getDeltaTime();
 void phase(int wheel, float inputPressure) ;
 float getVehicleSpeed() ;

 void cycleABS( float newInputPressure, float *brakeCMD[4], float *newWheelSpinVelocity[4], float newTimeStamp );

 #endif