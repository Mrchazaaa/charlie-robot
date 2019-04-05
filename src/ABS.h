 #ifndef ABS_H_
 #define ABS_H_


//threshold values above which ABS is activated
#define MIN_VEHICLE_VELOCITY_THRESHOLD 10
#define MIN_WHEEL_VELOCITY_THRESHOLD   10//20.4 //rads/sec
#define MIN_PRESSURE_THRESHOLD         13000
#define APPLY_DELAY                    0.05
#define PRIMARY_APPLY_RATE             10000000/*2500000*///6500000
#define SECONDARY_APPLY_RATE           13000000//11000000//9000000//5000000///*250000*/650000
#define RELEASE_RATE                   8750000//2000000//1300000  //Pa/sec
#define MIN_WHEEL_SPIN_ACCELERATION    -75//-100//-110//-150//-250//-2.5//-175 //rad/sec^2
#define MAX_WHEEL_SPIN_ACCELERATION    50//1//50 //rad/sec^2
#define MAX_WHEEL_SLIP                 0.09//0.1//0.12//0.15
#define MAX_BRAKE_PRESSURE             13000000 //Pa

 int getPhaseStates(int index) ;
 float getWheelSpinVelocity(int index) ;
 float getWheelSpinAcceleration(int index) ;
 float getWheelSlipAcceleration(int index) ;
 float getWheelSlip(int index) ;
 float getDeltaTime();
 void phase(int wheel) ;
 float getVehicleSpeed() ;

 void cycleABS( float newInputPressure, float *brakeCMD[4], float *newWheelSpinVelocity[4], float *newWheelSlipAcceleration[4], float newTimeStamp );

 #endif