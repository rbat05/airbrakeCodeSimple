#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "Dynamics.h"
#include "RocketVariables.h"
#include "ServoController.h"
#include "airbraketest.h"
#include "baro.h"
#include "ekf.h"
#include "hexdump.h"
#include "imu.h"

// Set to 1 to enable Hardware-In-The-Loop simulation, 0 for real sensors
#ifndef ENABLE_HITL
#define ENABLE_HITL 1
#endif

#if ENABLE_HITL
#include "hitl.h"
#endif

#define SAMPLE_RATE_MS 10  // 100 Hz
#define SERVO_PIN 16

#define AIRBRAKE_TEST 0

static Servo s_servo;
static int s_servoAngle = 0;
static int s_servoDirection = 1;
static uint32_t s_lastServoStepMs = 0;
static uint32_t s_profileStartMs = 0;
static uint32_t s_profileSamples = 0;
static uint32_t s_imuTimeUs = 0, s_baroTimeUs = 0, s_logTimeUs = 0;

float dt;
EKF ekf;
float velocity = 0;
float u_prev = 0.0;
float u = 0.0;
float h_pred = 0.0;

float gyro_yaw;        // rad/s
float accel_vertical;  // m/s^2, RAW body-axis accel (gravity still included)
float gyro_pitch;      // rad/s
float barometer_raw;   // metres

int loop_count = 0;

// ============================================================
// FLIGHT PHASE STATE MACHINE
//
// Drives how much we trust the barometer in ekf_update(). During motor
// burn the baro is unreliable (vibration + dynamic pressure effects),
// so instead of feeding it into the filter at full trust — or skipping
// the update outright, which would create a discontinuity when we
// re-enable it — we scale R up so the Kalman gain smoothly collapses
// toward zero for the duration of the burn, then relaxes back to
// nominal once burnout is confirmed.
// ============================================================

enum FlightPhase { PHASE_PAD, PHASE_BURN, PHASE_COAST };
static FlightPhase phase = PHASE_PAD;
static int confirm_count = 0;

const float LAUNCH_ACCEL_THRESHOLD = 15.0f;   // m/s^2 net vertical accel -> liftoff
const float BURNOUT_ACCEL_THRESHOLD = 2.0f;   // m/s^2 net vertical accel -> tailoff/burnout
const int CONFIRM_SAMPLES = 15;               // ~150ms @ 100Hz debounce, tune to motor tailoff
const float BURN_R_SCALE = 500.0f;            // how strongly to distrust baro during burn
const int RAMP_SAMPLES = 60;                  // ~0.6s @ 100Hz to ease trust back in after burnout
static int ramp_remaining = 0;

void updateFlightPhase(float accel_vertical_raw) {
  // Rough net vertical acceleration for phase detection only — deliberately
  // NOT orientation-compensated (that's what the EKF's internal a_net is
  // for). This just needs to reliably cross a threshold, not be precise.
  float a_net_approx = accel_vertical_raw - G;

  switch (phase) {
    case PHASE_PAD:
      if (a_net_approx > LAUNCH_ACCEL_THRESHOLD) {
        phase = PHASE_BURN;
        confirm_count = 0;
      }
      break;

    case PHASE_BURN:
      if (a_net_approx < BURNOUT_ACCEL_THRESHOLD) {
        confirm_count++;
        if (confirm_count > CONFIRM_SAMPLES) {
          phase = PHASE_COAST;
          ramp_remaining = RAMP_SAMPLES;  // start easing trust back in
        }
      } else {
        confirm_count = 0;  // reset on any sample that still looks like thrust
      }
      break;

    case PHASE_COAST:
      // Stays here for the rest of the flight. Re-arm PHASE_PAD here if
      // you need multi-boost / staged motor support.
      break;
  }

  // Snapping R straight back to nominal at burnout causes a one-shot
  // over-correction: R was inflated through the whole burn, so P grew
  // largely unchecked, and the baro itself reads low during high-speed
  // flight (dynamic pressure error at the static port). Ramp the trust
  // back in over RAMP_SAMPLES steps so that bias gets absorbed gradually
  // instead of in a single large jump.
  float r_scale;
  if (phase == PHASE_BURN) {
    r_scale = BURN_R_SCALE;
  } else if (ramp_remaining > 0) {
    float frac = (float)ramp_remaining / (float)RAMP_SAMPLES;
    r_scale = 1.0f + (BURN_R_SCALE - 1.0f) * frac;
    ramp_remaining--;
  } else {
    r_scale = 1.0f;
  }

  ekf_set_r_scale(&ekf, r_scale);
}

void setup() {
  Serial.begin(115200);
  delay(500);  // let the monitor connect
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Serial.println(
      "[MAIN] Initialising I2C sensors on Wire (SDA=GPIO33, SCL=GPIO32)");

  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32


#if AIRBRAKE_TEST
  RunAirbrakeSim();
  while (1);
#endif

#if ENABLE_HITL
  initHITL();
  bool imuOk = true;
  bool baroOk = true;
#else
  bool imuOk = initIMU();
  bool baroOk = initBaro();
#endif
  bool binOk = initBinLog();

  Serial.printf("[MAIN] IMU init: %s\n", imuOk ? "OK" : "FAIL");
  Serial.printf("[MAIN] Baro init: %s\n", baroOk ? "OK" : "FAIL");
  Serial.printf("[MAIN] BinLog init: %s\n", binOk ? "OK" : "FAIL");

  float h0 = 0;

  dt = 0.01f;  // 100Hz IMU rate (Can control to be loop rate)
  ekf_init(&ekf, dt, h0);

  phase = PHASE_PAD;
  confirm_count = 0;

  Serial.printf("[MAIN] Logging binary sensor data at %d Hz\n\n",
                1000 / SAMPLE_RATE_MS);
}

void loop() {
  loop_count++;
#if ENABLE_HITL
  updateHITL();
  IMUData imu = readIMUHITL();
  BaroData baro = readBaroHITL();
#else
  IMUData imu = readIMU();
  BaroData baro = readBaro();
#endif

  // EKF STEP
  static uint32_t prev_ms = millis();
  uint32_t now = millis();
  dt = (now - prev_ms) / 1000.0f;
  prev_ms = now;

  gyro_yaw = imu.gyroZ;     // rad/s
  accel_vertical = imu.accelY;  // m/s^2, RAW — gravity NOT subtracted here.
                                 // ekf_predict() removes gravity internally
                                 // (see a_net = a_true*cos*cos - G in ekf.c).
                                 // Subtracting it here too would remove it
                                 // TWICE and bias every predict step.
  gyro_pitch = imu.gyroX;   // rad/s

  // Raw INS-only velocity estimate, purely for comparison/logging against
  // the fused EKF output — NOT fed back into the filter.
  velocity += (accel_vertical - G) * dt;

  updateFlightPhase(accel_vertical);  // sets R scale for the update below

  ekf_predict(&ekf, accel_vertical, gyro_pitch,
              gyro_yaw);  // Predicts height from IMU
  if (loop_count == 4) {
    barometer_raw = baro.altitudeM;   // metres (Raw barometer estimate)
    ekf_update(&ekf, barometer_raw);  // Corrects prediction of height from IMU
                                      // from barometer reading. Trust in this
                                      // measurement is controlled by the R
                                      // scale set in updateFlightPhase() above.
    loop_count = 0;
  }

  // Serial.printf(
  //     "EKF Height: %.2f m, EKF Velocity: %.2f m/s, IMU Velocity: %.2f m/s, "
  //     "Baro Altitude: %.2f m, Phase: %d\n",
  //     ekf.x[0], ekf.x[1], velocity, barometer_raw, (int)phase);

  EKFData ekfData = setEKFData(ekf.x[0], ekf.x[1], velocity);

  // MPC STEP
  // Only activate airbrakes when above 100m altitude
  if (ekf.x[0] > 100.0) {
    u = OptimiseControlInputBinarySearchConstraint(ekf.x[0], velocity, u_prev,
                                                   0);
    SetServoAngle(u);  // u = 0–180 degrees

    // u: servo command
    // h_pred: predicted apogee from EKF state and current control input

    u_prev = u;
    h_pred = PredictApogee(ekf.x[0], velocity, u);
  }

  // Serial.printf(
  //     "Servo Command: %.2f degrees, "
  //     "Predicted Apogee: %.2f m\n",
  //     u, h_pred);

  ModelData modelData = setModelData(h_pred, u);
  logSensorsBin(imu, baro, modelData, ekfData);

  // Serial.printf(
  //     "Baro Alt: %.2f m, EKF Height: %.2f m, EKF Velocity: %.2f m/s, IMU
  //     Vel:"
  //     "%.2f m/s, Servo Cmd: %.2f deg, Pred Apogee: %.2f m, Phase: %d\n",
  //     barometer_raw, ekf.x[0], ekf.x[1], velocity, u, h_pred, (int)phase);

  delay(SAMPLE_RATE_MS);  // Do not remove, fucks up the timing and EKF
                          // convergence
}