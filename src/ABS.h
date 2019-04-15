 #ifndef ABS_H_
 #define ABS_H_

#define FL 0 //used to access wheel index for FRONT LEFT
#define FR 1 //used to access wheel index for FRONT RIGHT
#define RL 2 //used to access wheel index for REAR LEFT
#define RR 3 //used to access wheel index for REAR RIGHT

//threshold values above which ABS is activated
#define MIN_VEHICLE_VELOCITY_THRESHOLD 20
#define MIN_WHEEL_VELOCITY_THRESHOLD   20//20.4 //rads/sec
#define MIN_PRESSURE_THRESHOLD         13000
#define APPLY_DELAY                    0.04//0.05
#define PRIMARY_APPLY_RATE             11000000//10000000/*2500000*///6500000
#define SECONDARY_APPLY_RATE           8458000//13000000//11000000//9000000//5000000///*250000*/650000

#define RELEASE_RATE                   50000000//9500000 //Pa/sec

#define MIN_WHEEL_SPIN_ACCELERATION    -95//-80//-80//-100//-125//-100
#define MAX_WHEEL_SPIN_ACCELERATION    0//5//1//50 //rad/sec^2
#define MAX_WHEEL_SLIP                 0.1//0.13//0.11//0.09//0.1//0.12//0.15
#define MAX_BRAKE_PRESSURE             13000000 //Pa

 int getPhaseStates(int index) ;
 float getWheelSpinVelocity(int index) ;
 float getWheelSpinAcceleration(int index) ;
 float getWheelSlipAcceleration(int index) ;
 float getWheelSlip(int index) ;
 float getDeltaTime();
 void phase(int wheel) ;
 float getVehicleSpeed() ;

 void cycleABS( float newInputPressure, float *brakeCMD[4], float *newWheelSpinVelocity[4], float *newWheelSlipAcceleration[4], float newTimeStamp, float refSpeed );

int getPhaseStates(int index) { return phaseStates[index]; }
float getWheelSpinVelocity(int index) { return wheelSpinVelocity[index]; }
float getWheelSpinAcceleration(int index) { return wheelSpinAcceleration[index]; }
float getWheelSlipAcceleration(int index) { return wheelSlipAcceleration[index]; }
float getWheelSlip(int index) { return wheelSlip[index]; }
float getDeltaTime() { return deltaTime; }
float getVehicleSpeed() { return vehicleSpeed; }

 #endif