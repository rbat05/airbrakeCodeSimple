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
#define ENABLE_HITL 0
#endif

#if ENABLE_HITL
#include "hitl.h"
#endif

#define SAMPLE_RATE_MS 10  // 100 Hz
#define SERVO_PIN 16
#define SERVO_STEP_MS 1000

#define SERVO_MIN_ANGLE 63
#define SERVO_MAX_ANGLE 165

#define AIRBRAKE_TEST 0

static Servo s_servo;
static int s_servoAngle = SERVO_MIN_ANGLE;
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
float accel_vertical;  // m/s^2
float gyro_pitch;      // rad/s
float barometer_raw;   // metres

int loop_count = 0;

static void stepServo() {
  s_servo.write(s_servoAngle);

  s_servoAngle += s_servoDirection * 10;
  if (s_servoAngle >= SERVO_MAX_ANGLE) {
    s_servoAngle = SERVO_MAX_ANGLE;
    s_servoDirection = -1;
  } else if (s_servoAngle <= SERVO_MIN_ANGLE) {
    s_servoAngle = SERVO_MIN_ANGLE;
    s_servoDirection = 1;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // let the monitor connect
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Serial.println(
      "[MAIN] Initialising I2C sensors on Wire (SDA=GPIO33, SCL=GPIO32)");

  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32

  s_servo.setPeriodHertz(50);
  s_servo.attach(SERVO_PIN, 500, 2400);

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

  if (binOk) {
    Serial.println("[MAIN] SD Card successfully mounted and ready.");
  } else {
    Serial.println(
        "[MAIN] SD Card initialization FAILED. Check wiring/SD card.");
  }

  // Prints out sensor readings
  while (true) {
#if ENABLE_HITL
    updateHITL();
    IMUData imu = readIMUHITL();
    BaroData baro = readBaroHITL();
#else
    IMUData imu = readIMU();
    BaroData baro = readBaro();
#endif

    // Print all data on a single line
    Serial.printf(
        "[DEBUG] IMU: accel[%.2f, %.2f, %.2f] gyro[%.2f, %.2f, %.2f] | Baro: "
        "alt=%.2fm press=%.2fhPa | Servo: %d deg\n",
        imu.accelX, imu.accelY, imu.accelZ, imu.gyroX, imu.gyroY, imu.gyroZ,
        baro.altitudeM, baro.pressureHPa, s_servoAngle);

    stepServo();  // Steps by 10 degress, back and forth
    delay(100);   // Print at 10Hz so it doesn't flood the terminal too fast
  }

  float h0 = 0;

  dt = 0.01f;  // 100Hz IMU rate (Can control to be loop rate)
  ekf_init(&ekf, dt, h0);

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

  gyro_yaw = imu.gyroZ;                 // rad/s
  accel_vertical = imu.accelY - 9.81f;  // m/s^2
  gyro_pitch = imu.gyroX;               // rad/s

  velocity += accel_vertical * dt;  // Raw Velocity estimate
  ekf_predict(&ekf, accel_vertical, gyro_pitch,
              gyro_yaw);  // Predicts height from IMU
  if (loop_count == 4) {
    barometer_raw = baro.altitudeM;   // metres (Raw barometer estimate)
    ekf_update(&ekf, barometer_raw);  // Corrects prediction of height from IMU
                                      // from barometer reading
    loop_count = 0;
  }

  // Serial.printf(
  //     "EKF Height: %.2f m, EKF Velocity: %.2f m/s, IMU Velocity: %.2f m/s, "
  //     "Baro Altitude: %.2f m\n",
  //     ekf.x[0], ekf.x[1], velocity, barometer_raw);

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
  //     "%.2f m/s, Servo Cmd: %.2f deg, Pred Apogee: %.2f m\n",
  //     barometer_raw, ekf.x[0], ekf.x[1], velocity, u, h_pred);

  delay(SAMPLE_RATE_MS);  // Do not remove, fucks up the timing and EKF
                          // convergence
}
