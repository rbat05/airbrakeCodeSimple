#include "RocketVariables.h"


// ---- constants ----
float g = 9.81f;
float rho = 1.0f;


float A = 3.1415962f*(0.084f/2.0f)*(0.084f/2.0f);
float m_initial = 2.66f;
float m_final = 2.573f;
float burn_time = 1.8f;
float base_Cd = 0.5f;

// control
float href = 430.0f;
float u_max = 70.0f;
float u_min = 0.0f;


// global state
State_t state = {
   .h = 0.0f,
   .v = 0.0f
};

