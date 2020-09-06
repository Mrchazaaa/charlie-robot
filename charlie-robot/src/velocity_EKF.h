
#ifdef __cplusplus
// Tiny_EKF include
extern "C" {
  #include "tinyekf_config.h"
  #include "tiny_ekf.h"
}
#include <strings.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <string>
using namespace std;
// Eigen3 includes
//#include "Eigen/Core.h"
#include "Eigen/LU"
using namespace Eigen;
// Autodiff include
#include "autodiff/forward.hpp"
#include "autodiff/forward/eigen.hpp"
using namespace autodiff;
extern "C" {
#endif
    int start_ekf(double initial_velocity);
    
    double step_ekf(double driving_angle, double front_wheel_vel, double rear_wheel_vel);
#ifdef __cplusplus
} 
#endif

