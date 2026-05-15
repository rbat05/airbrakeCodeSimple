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
#include "loop_timer.h"  // ← add this

#ifndef ENABLE_HITL
#define ENABLE_HITL 1
#endif
#if ENABLE_HITL
#include "hitl.h"
#endif

#define SAMPLE_RATE_MS 10  // 50 Hz
#define SERVO_PIN 16
#define AIRBRAKE_TEST 0

// ── Print loop stats every N iterations
// ───────────────────────────────────────
#define STATS_EVERY 0  // ~2 s at 50 Hz; set 0 to disable

static LoopTimer loopTimer(SAMPLE_RATE_MS);

static Servo s_servo;
EKF ekf;
float dt;
float velocity = 0;
float u_prev = 0.0f;
float u = 0.0f;
float h_pred = 0.0f;
float gyro_yaw, accel_vertical, gyro_pitch, barometer_raw;
int loop_count = 0;

// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Wire.begin(33, 32);

  ServoInit();

#if AIRBRAKE_TEST
  RunAirbrakeSim();
  while (1);
#endif

#if ENABLE_HITL
  initHITL();
  bool imuOk = true, baroOk = true;
#else
  bool imuOk = initIMU();
  bool baroOk = initBaro();
#endif
  bool binOk = initBinLog();

  Serial.printf("[MAIN] IMU: %s  BARO: %s  BinLog: %s\n", imuOk ? "OK" : "FAIL",
                baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");

  ekf_init(&ekf, SAMPLE_RATE_MS / 1000.0f, 0.0f);
  Serial.printf("[MAIN] Target loop: %d ms (%d Hz)\n\n", SAMPLE_RATE_MS,
                1000 / SAMPLE_RATE_MS);
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  // ── Timing: measure real dt, sleep remaining budget at the bottom ─────────
  dt = loopTimer.tick();  // real dt in seconds; replaces millis() arithmetic
  loop_count++;

  // ── Sensor reads ──────────────────────────────────────────────────────────
#if ENABLE_HITL
  updateHITL();
  IMUData imu = readIMUHITL();
  BaroData baro = readBaroHITL();
#else
  IMUData imu = readIMU();
  BaroData baro = readBaro();
#endif

  // ── EKF ───────────────────────────────────────────────────────────────────
  gyro_yaw = imu.gyroZ;
  accel_vertical = imu.accelY - 9.81f;
  gyro_pitch = imu.gyroX;

  velocity += accel_vertical * dt;
  ekf_predict(&ekf, accel_vertical, gyro_pitch, gyro_yaw);

  if (loop_count >= 4) {
    barometer_raw = baro.altitudeM;
    ekf_update(&ekf, barometer_raw);
    loop_count = 0;
  }

  EKFData ekfData = setEKFData(ekf.x[0], ekf.x[1], velocity);

  // ── MPC ───────────────────────────────────────────────────────────────────
  if (ekf.x[0] > 100.0f) {
    u = OptimiseControlInputBinarySearchConstraint(ekf.x[0], velocity, u_prev,
                                                   0);
    SetServoAngle(u);
    u_prev = u;
    h_pred = PredictApogee(ekf.x[0], velocity, u);
  }

  ModelData modelData = setModelData(h_pred, u);
  logSensorsBin(imu, baro, modelData, ekfData);

  // ── Periodic stats print ──────────────────────────────────────────────────
#if STATS_EVERY > 0
  if (loop_count % STATS_EVERY == 1) {
    LoopStats s = loopTimer.stats();
    Serial.printf(
        "[LOOP] dt: %.2f ms | busy: %.2f ms | avg busy: %.2f ms | "
        "max busy: %.2f ms | overruns: %lu\n",
        s.dt_ms, s.busy_ms, s.avg_busy_ms, s.max_busy_ms, s.overruns);
  }
#endif

  // ── Sleep remaining budget to hit target rate ─────────────────────────────
  loopTimer.sleep();  // replaces delay(SAMPLE_RATE_MS)
}