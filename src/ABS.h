 #ifndef ABS_H_
 #define ABS_H_

 void phase(int wheel);

 void cycleABS( float newInputPressure, float *wheelBrakeCMD[4], float *newWheelSpinVelocity[4], float *newWheelSlipAcceleration[4], float newTimeStamp );

 #endif