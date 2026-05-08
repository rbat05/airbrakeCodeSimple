#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "baro.h"
#include "hexdump.h"
#include "imu.h"
#include "ekf.h"

#define SAMPLE_RATE_MS 10  // 1000 Hz
#define SERVO_PIN 16
#define SERVO_STEP_MS 1000

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

static void stepServo() {
  s_servo.write(s_servoAngle);

  s_servoAngle += s_servoDirection * 10;
  if (s_servoAngle >= 180) {
    s_servoAngle = 180;
    s_servoDirection = -1;
  } else if (s_servoAngle <= 0) {
    s_servoAngle = 0;
    s_servoDirection = 1;
  }
}

// Chuck all code that always runs in loop below the sensor reads
// Empty loop is ideal for profiling the overhead of the sensor reads + logging
static void loggingProfiler() {
  // Time IMU read
  uint32_t t0 = micros();
  IMUData imu = readIMU();
  s_imuTimeUs += (micros() - t0);

  // Time Baro read
  uint32_t t1 = micros();
  BaroData baro = readBaro();
  s_baroTimeUs += (micros() - t1);

  // Add whatever other functions that need to be ran here
  // EKF, Model Estimation, Servo control, etc.
  // stepServo();
  // delay(50);

  // Time logging
  uint32_t t2 = micros();
  logSensorsBin(imu, baro);
  s_logTimeUs += (micros() - t2);

  s_profileSamples++;  // Check this variables increment speed. Set
                       // s_profileSamples accordingly
  uint32_t nowMs = millis();

  if (s_profileStartMs == 0) {
    s_profileStartMs = nowMs;
  }

  uint32_t windowMs = nowMs - s_profileStartMs;
  if (s_profileSamples >= 100 && windowMs > 0) {
    float windowSec = windowMs / 1000.0f;
    float avgImuUs = s_imuTimeUs / static_cast<float>(s_profileSamples);
    float avgBaroUs = s_baroTimeUs / static_cast<float>(s_profileSamples);
    float avgLogUs = s_logTimeUs / static_cast<float>(s_profileSamples);
    float avgLoopUs = (avgImuUs + avgBaroUs + avgLogUs);
    float actualHz = s_profileSamples / windowSec;
    float maxFreqHz = 1e6f / avgLoopUs;

    Serial.println("\n[PROFILE] Time window:");
    Serial.printf("  Window: %.2f s, samples: %u\n", windowSec,
                  s_profileSamples);
    Serial.printf("  IMU read:    %.1f µs/sample\n", avgImuUs);
    Serial.printf("  Baro read:   %.1f µs/sample\n", avgBaroUs);
    Serial.printf("  Logging:     %.1f µs/sample\n", avgLogUs);
    Serial.printf("  Total overhead: %.1f µs/sample\n", avgLoopUs);
    Serial.printf("  Current rate: %.1f Hz\n", actualHz);
    Serial.printf("  Max achievable (no delay): %.0f Hz\n", maxFreqHz);

    // Buffer info
    uint16_t bufRecords = 100;  // BUFFER_RECORDS
    float bufTimeSec = bufRecords / actualHz;
    Serial.printf("  Buffer: %u records × 54 bytes = %u bytes\n", bufRecords,
                  bufRecords * 54);
    Serial.printf("  Flush interval: %.2f seconds at current rate\n",
                  bufTimeSec);

    s_profileStartMs = nowMs;
    s_profileSamples = 0;
    s_imuTimeUs = 0;
    s_baroTimeUs = 0;
    s_logTimeUs = 0;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);  // let the monitor connect
  // Serial.println("\n[MAIN] ESP32 Sensor Logger");
  // Serial.println(
  //     "[MAIN] Initialising I2C sensors on Wire (SDA=GPIO33, SCL=GPIO32)");

  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32

  s_servo.setPeriodHertz(50);
  s_servo.attach(SERVO_PIN, 500, 2400);
  stepServo();

  bool imuOk = initIMU();
  bool baroOk = initBaro();
  bool binOk = initBinLog();

  // Serial.printf("[MAIN] IMU init: %s\n", imuOk ? "OK" : "FAIL");
  // Serial.printf("[MAIN] Baro init: %s\n", baroOk ? "OK" : "FAIL");
  // Serial.printf("[MAIN] BinLog init: %s\n", binOk ? "OK" : "FAIL");
  
  float h0 = 0;
  
  dt= 0.1f;   // 100Hz IMU rate (Can control to be loop rate)
  ekf_init(&ekf, dt, h0);

  // if (!imuOk || !baroOk || !binOk) {
  //   Serial.println("[MAIN] !! One or more inits failed — check wiring !!");
  //   Serial.printf("  IMU: %s  BARO: %s  BIN: %s\n", imuOk ? "OK" : "FAIL",
  //                 baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");
  //   while (true) {
  //     delay(1000);
  //   }
  // }

  // Serial.printf("[MAIN] Logging binary sensor data at %d Hz\n\n",
  //               1000 / SAMPLE_RATE_MS);
}

void loop() {
  // loggingProfiler();

  IMUData imu = readIMU();
  BaroData baro = readBaro();
  logSensorsBin(imu, baro);
  delay(SAMPLE_RATE_MS);
  
  static uint32_t prev_ms = millis();
  uint32_t now = millis();
  dt = (now - prev_ms) / 1000.0f;
  prev_ms = now;
  //print all three accelerations and gyros to serial
  //float accel_z = imu.accelZ;    // m/s^2
  //float gyro_y = imu.gyroY;     // rad/s
  float gyro_yaw = imu.gyroZ;     // rad/s
  float accel_vertical = imu.accelY - 9.81f;    // m/s^2
  // float accel_x = imu.accelX;    // m/s^2
  float gyro_pitch = imu.gyroX;     // rad/s
  float barometer_raw = baro.altitudeM + 42;   // metres

  ekf_predict(&ekf, accel_vertical, gyro_pitch, gyro_yaw);
  // float barometer_raw = baro.altitudeM;   // metres
  ekf_update(&ekf, barometer_raw);
  
  Serial.printf(
    "%lu,%.3f,%.3f,%.3f,%.3f\n",
    millis(),
    barometer_raw,
    velocity,
    ekf.x[0],
    ekf.x[1]
  );
  delay(100);
  
}
