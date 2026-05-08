#include "RocketVariables.h"


// ---- constants ----
float g = 9.81f;
float rho = 1.0f;


float A = 3.1415962f*(0.084f/2.0f)*(0.084f/2.0f);
float m_initial = 2.5f;
float m_final = 2.0f;
float burn_time = 1.8f;
float base_Cd = 0.5f;
float Cd_active = 0.5f;


// time steps
// float loop_dt = 0.015;
float loop_dt = 0.01;


// float predict_dt = 0.02;
float predict_dt = 0.5;
float predict_dt_final = 0.1;


// control
float href = 475.0f;
float u_max = 70.0f;
float u_min = 0.0f;






float active_time_offset = 0.5f;


// global state
State_t state = {
   .h = 0.0f,
   .v = 0.0f
};

