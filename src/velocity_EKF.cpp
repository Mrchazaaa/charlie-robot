#include "velocity_EKF.h"

int stepCounter;

static ekf_t ekf;

static void init(ekf_t * ekf, double initial_velocity)
{
    stepCounter = 0;

    //initialize Q matrix (model noise covariance)
    double Q0 = 0.1;
    double Q[3][3] = { {Q0 , 0  , 0  },
                       {0  , Q0 , 0  },
                       {0  , 0  , Q0 }};
    memcpy( ekf->Q, Q, sizeof(Q));

    double P0 = 1000000;
    double P[3][3] = { {P0 , 0  , 0  },
                       {0  , P0 , 0  },
                       {0  , 0  , P0 }};
    memcpy( ekf->P, P, sizeof(P));
    
    //initialize R matrix (observation noise covariance)
    double R0[3] = { 0.00001, 0.01, 0.01 };
    double R[3][3] = { {R0[0] , 0     , 0     },
                       {0     , R0[1] , 0     },
                       {0     , 0     , R0[2] } };
    memcpy( ekf->R, R, sizeof(R));

    double e = 0.01;
    ekf->x[0] = initial_velocity - e;
    ekf->x[1] = e;
    ekf->x[2] = 0.17;
}

//simulation parameters
dual m   = 1265.0;
dual mb  = 1209;
dual mf  = 28;
dual mr  = 28;
dual Lf  = 1;
dual Lr  = 1.45;
/* dual Iz  = 1627; */
dual Iz  = 2927;
dual hf  = 0.53;
dual hr  = 0.52;
dual Rw  = 0.3179;
double Rwh  = 0.3179;
dual Iw  = 4.07;
dual Ksf = 30000;
dual Ksr = 35000;
dual Bsf = 5000;
dual Bsr = 4500;
dual Ip  = 2035;
dual Kb  = 0.3;
dual g   = 9.81;
//Burckhardt coefficients
/* dual c1  = 1; */
/* dual c2  = 26; */
/* dual c3  = 0.25; */
dual c1  = 1.28;
dual c2  = 23.99;
dual c3  = 0.52;

//input vector w
VectorXdual w(3);

bool runOnce;

/* #define debug */
//#define output
#define ekf_run

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

    //update fx (state transitions)
    VectorXdual z(3);

    z << x(0) + x_velocity_deriv, 
         x(1) + y_velocity_deriv, 
         x(2); 
    
    if (runOnce) {
    #ifdef output
    std::cout << alphaf << "," << alphar << ","
              << vr << "," << vf << ","
              << Sxf << "," << Sxr << ","
              << Syf << "," << Syr << ","
              << Sresf << "," << Sresr << ","
              << Fzf << "," << Fzr << ","
              << Muresf << "," << Muresr << ","
              << Fxf << "," << Fyf << ","
              << Fxr << "," << Fyr << ","
              << x_velocity_deriv << ","
              << z(0) << "," 
              << z(1) << ","
              << z(2);
    #endif
    #ifdef debug
    std::cout << "Model: \n";
    std::cout << "alphaf         : " << alphaf << "\n";
    std::cout << "alphar         : " << alphar << "\n";
    std::cout << "vf             : " << vf << "\n";
    std::cout << "vr             : " << vr << "\n";
    std::cout << "Sxf            : " << Sxf << "\n";
    std::cout << "Sxr            : " << Sxr << "\n";
    std::cout << "Syf            : " << Syf << "\n";
    std::cout << "Syr            : " << Syr << "\n";
    std::cout << "Sresf          : " << Sresf << "\n";
    std::cout << "Sresr          : " << Sresr << "\n";
    std::cout << "Fzf            : " << Fzf << "\n";
    std::cout << "Fzr            : " << Fzr << "\n";
    std::cout << "Muresf         : " << Muresf << "\n";
    std::cout << "Muresr         : " << Muresr << "\n";
    std::cout << "Fxf            : " << Fxf << "\n";
    std::cout << "Fyf            : " << Fyf << "\n";
    std::cout << "Fxr            : " << Fxr << "\n";
    std::cout << "Fyr            : " << Fyr << "\n";
    std::cout << "x_vel_der      : " << x_velocity_deriv << "\n";
    std::cout << "y_vel_der      : " << y_velocity_deriv << "\n";
    std::cout << "----------------------------------------------------" << std::endl;;
    #endif
    }
    runOnce = false;

    return z;
}

VectorXdual measurement(VectorXdual& x) {
    /* dual newalphaf = w(0) - atan( (x(1) + (Lf * x(2))/x(0) )); */
    /* dual newalphar = -atan( (x(1) - (Lr * x(2)))/x(0) ); */
    /* dual newvf = sqrt( pow( x(1) + (Lf * x(2)) , 2) + pow(x(0), 2)); */
    /* dual newvr = sqrt( pow( x(1) - (Lr * x(2)) , 2) + pow(x(0), 2)); */

    /* dual newSxf = (1/newvf)*((Rw * w(1) * cos(newalphaf)) - newvf) ; */
    /* dual newSyf = (1/newvf)*(Rw * w(1) * sin(newalphaf)) ; */
    /* dual newSxr = (1/newvr)*((Rw * w(2) * cos(newalphar)) - newvr) ; */
    /* dual newSyr = (1/newvr)*(Rw * w(2) * sin(newalphar)) ; */

    /* //update measurement function */
    /* /1* dual hx0 = newalphaf + atan( (x(1) + (Lf*x(2))) / x(0) ); *1/ */
    /* dual hx0 = w(0); */
    /* dual hx1 = (newSyf*newvf)/(Rw*sin(newalphaf)); */
    /* dual hx2 = (newSyr*newvr)/(Rw*sin(newalphar)); */

    /* //update hx (state transitions) */
    /* VectorXdual z(3); */
    /* z << hx0, hx1, hx2; */
    
    return w;
}

static void model(ekf_t * ekf)
{
    stepCounter++;
    runOnce = true; //used for debugging the state transition function REMOVE

    //calculate state transition 
    VectorXdual x(3);
    x << ekf->x[0], ekf->x[1], ekf->x[2];

    #ifdef debug
    std::cout << "==========================new step " << stepCounter << "====================" << std::endl;;
    std::cout << "Data: \n";
    std::cout << "driving_angle     : " << w(0) << "\n";
    std::cout << "front_wheel_vel   : " << w(1) << " (" << w(1).val * Rwh << ") \n";
    std::cout << "rear_wheel_vel    : " << w(2) << " (" << w(2).val * Rwh << ") \n";
    std::cout << "--------------------------------------------------------" << std::endl;;
    std::cout << "input X" << stepCounter << ": \n";
    std::cout << "X_velocity        : " << x(0) << "\n";
    std::cout << "Y_velocity        : " << x(1) << "\n";
    std::cout << "scalar            : " << x(2) << "\n";
    std::cout << "--------------------------------------------------------" << std::endl;;
    #endif

    //global input vector w
    /* w << front_wheel_vel, rear_wheel_vel, driving_angle; */
    /* w << driving_angle, front_wheel_vel, rear_wheel_vel; */

    //calculate output vector u 
    VectorXdual u = stateTransition(x);

    //update transition function
    ekf->fx[0] = u(0).val;
    ekf->fx[1] = u(1).val;
    ekf->fx[2] = u(2).val;

    //calculate jacobian matrix du/dx
    MatrixXd dudx = jacobian( stateTransition, u, x);

    //update F matrix (jacobian of process model)
    double F[3][3];

    int i, j;

    for (j=0; j<3; ++j) {for (i=0; i<3; ++i) {F[i][j] = dudx(i, j);}}
    memcpy( ekf->F, F, sizeof(F));

    //calculate output vector u 
    /* VectorXdual z = measurement(x); */
    VectorXdual z = measurement(u);
    
    //update measurement function
    ekf->hx[0] = z(0).val;
    ekf->hx[1] = z(1).val;
    ekf->hx[2] = z(2).val;

    //calculate jacobian matrix dz/dx
    MatrixXd dzdx = jacobian( measurement, z, u);

    double H[3][3];
    
    for (j=0; j<3; ++j) {for (i=0; i<3; ++i) {H[i][j] = dzdx(i, j);}}
    memcpy( ekf->H, H, sizeof(H));

    #ifndef ekf_run
    ekf->x[0] = ekf->fx[0];
    ekf->x[1] = ekf->fx[1];
    ekf->x[2] = ekf->fx[2];
    ekf->x[3] = ekf->fx[3];
    ekf->x[4] = ekf->fx[4];
    ekf->x[5] = ekf->fx[5];
    ekf->x[6] = ekf->fx[6];
    ekf->x[7] = ekf->fx[7];
    #endif

    #ifdef debug
    std::cout << "output X" << stepCounter << ": \n";
    std::cout << "X_velocity        : " << ekf->x[0] << "\n";
    std::cout << "Y_velocity        : " << ekf->x[1] << "\n";
    std::cout << "scalar            : " << ekf->x[2] << "\n";
    std::cout << "--------------------------------------------------------" << std::endl;;
    std::cout << "hx: \n";
    std::cout << "driving_angle     : " << z(0) << "\n";
    std::cout << "front_wheel_vel   : " << z(1) << "\n";
    std::cout << "rear_wheel_vel    : " << z(2) << "\n";
    std::cout << "--------------------------------------------------------" << std::endl;;
    std::cout << std::endl;
    #endif
}

int step_ekf(double driving_angle, double front_wheel_vel, double rear_wheel_vel){

    w << driving_angle, front_wheel_vel, rear_wheel_vel;

    double input[3];

    input[0] = w(0).val;
    input[1] = w(1).val;
    input[2] = w(2).val;

    model(&ekf);

    #ifdef ekf_run
    ekf_step(&ekf, input);
    #endif

    return ekf.x[0];
}

int start_ekf(double initial_velocity)
{   
    std::cout << "1" << std::endl;

    // Do generic EKF initialization
    ekf_init(&ekf, Nsta, Mobs);

    std::cout << "2" << std::endl;

    // Do local initialization
    init(&ekf, initial_velocity);

    return 0;
}
