#include "velocity_EKF.h"

//Generic Extended Kalman Filter implementation provided by TinyEKF
static ekf_t ekf;

//Local initialization Etended Kalman Filter with initial state values and covariance matrices
static void init(double initial_velocity)
{
    //initialize Q matrix (model noise covariance)
    double Q0 = 0.1;
    double Q[3][3] = { {Q0 , 0  , 0  },
                       {0  , Q0 , 0  },
                       {0  , 0  , Q0 }};
    memcpy( ekf.Q, Q, sizeof(Q));

    double P0 = 1000000;
    double P[3][3] = { {P0 , 0  , 0  },
                       {0  , P0 , 0  },
                       {0  , 0  , P0 }};
    memcpy( ekf.P, P, sizeof(P));
    
    //initialize R matrix (observation noise covariance)
    double R0[3] = { 0.00001, 0.01, 0.01 };
    double R[3][3] = { {R0[0] , 0     , 0     },
                       {0     , R0[1] , 0     },
                       {0     , 0     , R0[2] } };
    memcpy( ekf.R, R, sizeof(R));

    //Initialize state vector
    double e = 0.01;
    ekf.x[0] = initial_velocity - e;
    ekf.x[1] = e;
    ekf.x[2] = 0.17;
}

//simulation parameters
dual m   = 1265.0;
dual Rw  = 0.3179;
dual g   = 9.81;
//Burckhardt coefficients
dual c1  = 1.28;
dual c2  = 23.99;
dual c3  = 0.52;

//input vector w (driving angle, front wheel angular velocity, rear wheel angular velocity)
VectorXdual w(3);

//update state vector
VectorXdual stateTransition(VectorXdual& x) {

    dual alphaf = w(0) - atan( x(1) /x(0) );
    dual alphar = -atan( x(1)/x(0) );

    dual vf = sqrt( pow( x(1) , 2) + pow(x(0), 2));
    dual vr = sqrt( pow( x(1) , 2) + pow(x(0), 2));

    dual Sxf = -((Rw * w(1) * cos(alphaf)) - vf)/vf;
    dual Sxr =  ((Rw * w(2) * cos(alphar)) - vr)/vr; 

    dual Syf = (w(1)*Rw * sin(alphaf))/vf; 
    dual Syr = (w(2)*Rw * sin(alphar))/vr; 

    dual Sresf = sqrt( pow(Sxf, 2) + pow(Syf, 2) );
    dual Sresr = sqrt( pow(Sxr, 2) + pow(Syr, 2) );

    dual Fzf = (m/2)*g;
    dual Fzr = (m/2)*g;

    dual Muresf = ((c1*(1 - exp(-c2*Sresf) )) - (c3*Sresf) ) * x(2);
    dual Muresr = ((c1*(1 - exp(-c2*Sresr) )) - (c3*Sresr) ) * x(2);

    dual Fxf;
    dual Fyf;
    if (Sresf > 0.001 ){
        Fxf = (Muresf/Sresf)*Fzf*Sxf;
        Fyf = (Muresf/Sresf)*Fzf*Syf;
    } else {
        Fxf = 0;
        Fyf = 0;
    }

    dual Fxr;
    dual Fyr;
    if (Sresr > 0.001 ){
        Fxr = (Muresr/Sresr)*Fzr*Sxr;
        Fyr = (Muresr/Sresr)*Fzr*Syr;
    } else {
        Fxr = 0;
        Fyr = 0;
    }

    dual x_velocity_deriv = (( (Fxf*cos(w(0))) - (Fyf*sin(w(0))) + Fxr )/m) ;
    dual y_velocity_deriv = (( (Fyf*cos(w(0))) + (Fxf*sin(w(0))) + Fyr )/m) ; 

    //update fx (state transitions vector)
    VectorXdual z(3);

    z << x(0) + x_velocity_deriv, 
         x(1) + y_velocity_deriv, 
         x(2); 

    return z;
}

//update measurement prediction vector (assume zero derivative)
VectorXdual measurement(VectorXdual& x) {
    
    return w;
}

double step_ekf(double driving_angle, double front_wheel_vel, double rear_wheel_vel){

    //update input vector
    w << driving_angle, front_wheel_vel, rear_wheel_vel;

    //calculate state transition 
    VectorXdual x(3);
    x << ekf.x[0], ekf.x[1], ekf.x[2];

    //calculate output vector u 
    VectorXdual u = stateTransition(x);

    //update transition function
    ekf.fx[0] = u(0).val;
    ekf.fx[1] = u(1).val;
    ekf.fx[2] = u(2).val;

    //calculate jacobian matrix du/dx
    MatrixXd dudx = jacobian( stateTransition, u, x);

    //copy jacobian matrix into TinyEKF (jacobian of process model)
    double F[3][3];
    int i, j;
    for (j=0; j<3; ++j) {for (i=0; i<3; ++i) {F[i][j] = dudx(i, j);}}
    memcpy( ekf.F, F, sizeof(F));

    //predict measurement values
    VectorXdual z = measurement(u);
    
    //copy measurement function output into TinyEKF
    ekf.hx[0] = z(0).val;
    ekf.hx[1] = z(1).val;
    ekf.hx[2] = z(2).val;

    //calculate jacobian matrix dz/dx
    MatrixXd dzdx = jacobian( measurement, z, u);

    //copy jacobian matrix into TinyEKF (jacobian of measurement model)
    double H[3][3];
    for (j=0; j<3; ++j) {for (i=0; i<3; ++i) {H[i][j] = dzdx(i, j);}}
    memcpy( ekf.H, H, sizeof(H));

    //copy input vector into TinyEKF and calculate x state predictions
    double input[3];
    input[0] = w(0).val;
    input[1] = w(1).val;
    input[2] = w(2).val;

    //debugging
    //std::cout << "input X" << std::endl;
    //std::cout << x << std::endl;
    //std::cout << "===================================" << std::endl;
    
    //debugging
    //std::cout << "calculated X" << std::endl;
    //std::cout << u << std::endl;
    //std::cout << "===================================" << std::endl;

    //use TinyEKF to predict new state vector
    ekf_step(&ekf, input);

    //debugging
    //std::cout << "predicted X" << std::endl;
    //std::cout << x << std::endl;
    //std::cout << "===================================" << std::endl;

    //return predicted longitudinal velocity
    return ekf.x[0];
}

//reset EKF (called at the beginning of each braking procedure)
int start_ekf(double initial_velocity)
{   
    // Do generic EKF initialization
    ekf_init(&ekf, Nsta, Mobs);

    // Do local EKF initialization
    init(initial_velocity);

    return 0;
}
