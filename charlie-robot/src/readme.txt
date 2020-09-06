
# to compile this code include the relevant directories and std=c++1z or c++17
g++ velocity_EKF.cpp tiny_ekf.c -I autodiff/ -I eigen-git-mirror/ -std=c++1z -o tester
