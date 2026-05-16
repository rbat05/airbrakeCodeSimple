#ifndef ROCKETVARIABLES_H
#define ROCKETVARIABLES_H

// ---- constants (extern only) ----
extern float g;
extern float rho;

extern float A;
extern float m_initial;
extern float m_final;
extern float burn_time;
extern float base_Cd;

// control
extern float href;
extern float u_max;
extern float u_min;

// ---- state struct ----
typedef struct {
    float h;
    float v;
} State_t;

// global state
extern State_t state;

#endif