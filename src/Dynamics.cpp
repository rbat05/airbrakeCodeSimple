#include "Dynamics.h"

#include <Arduino.h>
#include <math.h>
#include <stdio.h>

#include "RocketVariables.h"
#include "ServoController.h"
#include "esp_timer.h"

// ---- helpers ----
float ABS(float a) { return (a < 0.0f) ? -a : a; }
float MIN(float a, float b) { return (a < b) ? a : b; }
float MAX(float a, float b) { return (a > b) ? a : b; }

// ---- dynamics ----
float f1(float s1, float s2) { return s2; }

float f2(float s1, float s2, float Ft, float Cd, float m) {
  return (1.0f / m) * (Ft - 0.5f * rho * Cd * A * s2 * s2 - m * g);
}

// RK4
void rk4(float* s1, float* s2, float Ft, float Cd, float m, float dt) {
  float k1_1 = f1(*s1, *s2);
  float k1_2 = f2(*s1, *s2, Ft, Cd, m);

  float k2_1 = f1(*s1 + 0.5f * dt * k1_1, *s2 + 0.5f * dt * k1_2);
  float k2_2 = f2(*s1 + 0.5f * dt * k1_1, *s2 + 0.5f * dt * k1_2, Ft, Cd, m);

  float k3_1 = f1(*s1 + 0.5f * dt * k2_1, *s2 + 0.5f * dt * k2_2);
  float k3_2 = f2(*s1 + 0.5f * dt * k2_1, *s2 + 0.5f * dt * k2_2, Ft, Cd, m);

  float k4_1 = f1(*s1 + dt * k3_1, *s2 + dt * k3_2);
  float k4_2 = f2(*s1 + dt * k3_1, *s2 + dt * k3_2, Ft, Cd, m);

  *s1 += (dt / 6.0f) * (k1_1 + 2 * k2_1 + 2 * k3_1 + k4_1);
  *s2 += (dt / 6.0f) * (k1_2 + 2 * k2_2 + 2 * k3_2 + k4_2);
}

// NEED TO REPLACE WITH IMPROVED EULER

// Getting Cd as a function of control input
float GetCd(float u) {
  float min_Cd = base_Cd;
  float max_Cd = 1.5f;

  float Cd = (max_Cd - min_Cd) / 70.0f * u + min_Cd;
  Cd = MIN(max_Cd, MAX(min_Cd, Cd));

  return Cd;
}

// this funnction predicts apogee of the rocket given current altitude,
// velocity, and control input

// neeed to improve this to take into account when the airbrakes will actually
// be at given deployment (lag)
float PredictApogee(float h_sim_predict, float v_sim_predict, float u) {
  // finding dt based on altitude
  float h_burnout = 120.0f;
  float h_final = href;

  float dt = (predict_dt - predict_dt_final) / (h_burnout - h_final) *
                 (h_sim_predict - h_burnout) +
             predict_dt;
  dt = MIN(predict_dt, MAX(dt, predict_dt_final));

  // dt   // slightly bigger for speed - smaller more accurate - doesnt have to
  // be same as control loop
  float Cd = GetCd(u);  // replace with cfd lookup

  while (v_sim_predict > 0.0f) {
    // rk4(&h_sim_predict, &v_sim_predict, 0.0f, Cd, m_final, predict_dt);
    rk4(&h_sim_predict, &v_sim_predict, 0.0f, Cd, m_final, dt);
  }

  return h_sim_predict;
}

// this functions finds the control input that minimising the error between
// predicted apogee and target apogee need to update to use either gradient
// descent or binary search for faster computation time
float OptimiseControlInput(float h, float v, float u_prev) {
  int iterations = 100;
  float min_objectiveVal = 1e9f;
  float u_opt = u_prev;
  float alpha = 0.5f;

  for (int i = 0; i < iterations; i++) {
    float u = (u_max - u_min) / ((float)iterations - 1.0f) * (float)i;

    float apogee = PredictApogee(h, v, u);

    float objectiveVal = alpha * (href - apogee) * (href - apogee) +
                         (1.0f - alpha) * (u - u_prev) * (u - u_prev);

    if (objectiveVal < min_objectiveVal) {
      min_objectiveVal = objectiveVal;
      u_opt = u;
    }
  }

  return u_opt;
}

// this optimiser is run at each timestep to find the optimal control input
// that minimises error between href and predicted apogee
// it uses binary search
float OptimiseControlInputBinarySearch(float h, float v, float u_prev) {
  // need to use servo speed as constraint - optimisation will change alot
  int optimiser_array_length = 20;
  float min_error = 500.0;
  float u_opt = u_prev;

  int L = 0;
  int R = optimiser_array_length - 1;
  int m;
  while (1) {
    if (L > R) {
      return u_opt;
    }

    m = L + (int)(((float)R - (float)L) / 2.0f);
    float u =
        (u_max - u_min) / ((float)optimiser_array_length - 1.0f) * (float)m;
    float error = href - PredictApogee(h, v, u);
    if (error < 0.0) {
      if (ABS(error) < min_error) {
        u_opt = u;
      }
      L = m + 1;
    } else {
      if (ABS(error) < min_error) {
        u_opt = u;
      }
      R = m - 1;
    }
  }

  return u_opt;
}

// this optimiser does the same as above but takes into acconut servo speed
// limits
float OptimiseControlInputBinarySearchConstraint(float h, float v, float u_prev,
                                                 int iteration) {
  // need to use servo speed as constraint - optimisation will change alot
  int optimiser_array_length = 21;
  float min_error = 500.0;
  float u_opt = u_prev;

  int length = optimiser_array_length - 1;
  int L = 0;
  int R = length;
  int m;
  float udt_max =
      (60.0 / 0.2) * loop_dt;  // need to update to include gear ratio

  float u_constrained_max = MIN(u_max, u_prev + udt_max);
  float u_constrained_min = MAX(0, u_prev - udt_max);

  float u_range = u_constrained_max - u_constrained_min;

  // debug print
  if (iteration == 0 || iteration == 1 || iteration == 2 || iteration == 3 ||
      iteration == 4 || iteration == 5 || iteration == 6) {
    //  Serial.printf("udt_max: %.3f ,uprev = %.3f, u_min = %.3f, umax =
    //  %.3f\n",udt_max, u_prev, u_constrained_min, u_constrained_max);
  }
  int count = 0;

  while (1) {
    if (L > R) {
      // printf("count: %d\n",count);
      u_opt = MIN(u_max, MAX(u_min, u_opt));
      return u_opt;
    }

    m = L + (int)(((float)R - (float)L) / 2.0f);

    // u represented as a linear fit of array index
    float u =
        (u_range / ((float)(length))) * (float)(m - length) + u_constrained_max;

    float error = href - PredictApogee(h, v, u);

    if (error < 0.0) {
      if (ABS(error) < min_error) {
        u_opt = u;
      }
      L = m + 1;
    } else {
      if (ABS(error) < min_error) {
        u_opt = u;
      }
      R = m - 1;
    }
    count++;
  }

  u_opt = MIN(u_max, MAX(u_min, u_opt));

  return u_opt;
}

// ---  embedded functions

// this functions returns 1 if launch is detected
// should use interrupt
int LaunchDetected() { return 1; }

// this function returns 1 if coast phase has been detected
// uses both accelerometer data and time since launch - can place weightings on
// each
// - should use interrupt
int CoastDetected(int launch_time) {
  // need to update to use accelerometer
  int time_since_launch = esp_timer_get_time() - launch_time;
  if (time_since_launch >
      (float)((burn_time + active_time_offset) * 1000000.0f)) {
    return 1;
  } else {
    return 0;
  }
}

// this function returns 1 if apogee is detected
// should use interrupt
int ApogeeDetected() {
  // need to update to only return 1 if velocity is held near 0 for certain time
  if (state.v < 0.5 && state.h > 100.0) {
    SetServoAngle(0.0);  // u = 0–180 degrees
    return 1;
  } else {
    return 0;
  }
}

ModelData setModelData(float predictedApogeeM, float servoCommand) {
  ModelData d;
  d.predictedApogeeM = predictedApogeeM;
  d.servoCommand = servoCommand;
  return d;
}

// i want control loop to run at fixed frequency - using timer/interupt?
// need to log altitude, velocity, orientation, control input, predicted apogee,
// drag coefficent to sd card - ill do later
