#ifndef EKF_H
#define EKF_H

// ============================================================
// ekf.h — Public interface for the Extended Kalman Filter
//
// This file contains ONLY:
//   - Constants and sensor parameters (#define)
//   - The EKF struct definition (types)
//   - Function declarations (signatures only, no code)
//
// The actual function code lives in ekf.c
// Any file that needs the EKF should #include "ekf.h"
// ============================================================

// ============================================================
// STATE VECTOR: x = [h, h_dot, theta_p, theta_y, b_gyro_p, b_gyro_y, b_acc]^T
//
//   x[0] = h          vertical height above launch pad (m)
//   x[1] = h_dot      vertical velocity, positive = upward (m/s)
//   x[2] = theta_p    pitch angle from vertical (rad)
//   x[3] = theta_y    yaw angle from vertical (rad)
//   x[4] = b_gyro_p   pitch gyro bias (rad/s)
//   x[5] = b_gyro_y   yaw gyro bias (rad/s)
//   x[6] = b_acc      accelerometer bias on body vertical axis (m/s^2)
// ============================================================

#define EKF_N 7  // number of states
#define G 9.81f  // gravity (m/s^2)

// ============================================================
// SENSOR PARAMETERS — MPU-6050 + BME280
//
// Used in ekf.c to compute Q and R from first principles.
// Adjust ACCEL_SAMPLE_RATE and GYRO_SAMPLE_RATE to match
// the rate you call ekf_predict() in your main loop.
// ============================================================

#define ACCEL_NOISE_DENSITY 400e-6f   // g/sqrt(Hz)   — MPU-6050 datasheet
#define ACCEL_BIAS_INSTABILITY 0.01f  // g            — bias drift per step
#define ACCEL_SAMPLE_RATE 100.0f      // Hz           — your IMU loop rate

#define GYRO_NOISE_DENSITY 0.005f   // deg/s/sqrt(Hz) — MPU-6050 datasheet
#define GYRO_BIAS_INSTABILITY 0.1f  // deg/s          — bias drift per step
#define GYRO_SAMPLE_RATE 100.0f     // Hz

#define BARO_RMS_NOISE 1.7f  // metres RMS — BME280 0.2hPa noise spec

// ============================================================
// EKF STRUCT
//
// Holds all filter state. Declare one globally in main and
// pass a pointer into every EKF function call.
// ============================================================

typedef struct {
  float x[EKF_N];         // state vector — current best estimate
  float P[EKF_N][EKF_N];  // covariance matrix — uncertainty about x
  float Q[EKF_N][EKF_N];  // process noise matrix — IMU noise per step
  float R;                // measurement noise — baro variance (m^2)
  float dt;               // predict step timestep (s)
} EKF;

// For data logging
struct EKFData {
  float filtered_height;    // x[0]
  float filtered_velocity;  // x[1]
  float imu_velocity_prediction;
};

EKFData setEKFData(float filtered_height, float filtered_velocity,
                   float imu_velocity_prediction);

// ============================================================
// PUBLIC FUNCTION DECLARATIONS
//
// These are the only functions visible outside ekf.c
// The matrix utility functions are internal to ekf.c and
// are NOT declared here — they are private implementation details.
// ============================================================

// Initialise the filter. Call once at startup.
//   ekf        : pointer to your EKF instance
//   dt_seconds : time between ekf_predict() calls (e.g. 0.01 for 100Hz)
//   h0         : initial height in metres from first baro reading
void ekf_init(EKF* ekf, float dt_seconds, float h0);

// Predict step. Call every IMU sample (fast loop).
//   a_body  : accelerometer body vertical axis (m/s^2) — az converted from raw
//   w_pitch : pitch gyro rate (rad/s)                 — gy converted from raw
//   w_yaw   : yaw gyro rate (rad/s)                   — gz converted from raw
//
// Convert raw MPU-6050 values before calling:
//   a_body  = az_raw * (9.81f / 16384.0f)
//   w_pitch = gy_raw * (M_PI / (180.0f * 131.0f))
//   w_yaw   = gz_raw * (M_PI / (180.0f * 131.0f))
void ekf_predict(EKF* ekf, float a_body, float w_pitch, float w_yaw);

// Update step. Call when a new baro reading is available (slow loop).
//   z_baro : barometer height (m) relative to pad
//            must use same reference pressure as h0 in ekf_init()
void ekf_update(EKF* ekf, float z_baro);

// Accessors — read estimated states after each predict/update call
float ekf_get_height(EKF* ekf);    // metres above pad
float ekf_get_velocity(EKF* ekf);  // m/s, positive = upward
float ekf_get_pitch(EKF* ekf);     // radians from vertical
float ekf_get_yaw(EKF* ekf);       // radians from vertical
float ekf_get_b_gyro_p(EKF* ekf);  // pitch gyro bias (rad/s)
float ekf_get_b_gyro_y(EKF* ekf);  // yaw gyro bias (rad/s)
float ekf_get_b_acc(EKF* ekf);     // accel bias (m/s^2)

#endif  // EKF_H