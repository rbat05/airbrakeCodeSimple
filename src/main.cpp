#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

#include "Dynamics.h"
#include "RocketVariables.h"
#include "ServoController.h"
#include "baro.h"
#include "ekf.h"
#include "hexdump.h"
#include "imu.h"

#define SAMPLE_RATE_MS 10  // 50 Hz
#define SERVO_PIN 16

// ── Print loop stats every N iterations
// ───────────────────────────────────────
#define STATS_EVERY 0  // ~2 s at 50 Hz; set 0 to disable

static Servo s_servo;
EKF ekf;
float dt;
float velocity = 0;
float u_prev = 0.0f;
float u = 0.0f;
float h_pred = 0.0f;
float gyro_yaw, accel_vertical, gyro_pitch, barometer_raw;
int loop_count = 0;

// ── FreeRTOS Dual-Core Logging ──────────────────────────────────────────────
struct LogMessage {
  IMUData imu;
  BaroData baro;
  ModelData modelInfo;
  EKFData ekf;
};

static QueueHandle_t logQueue = NULL;

static void sdLoggingTask(void* pvParameters) {
  LogMessage msg;
  while (true) {
    // Block indefinitely until a message arrives in the queue
    if (xQueueReceive(logQueue, &msg, portMAX_DELAY) == pdPASS) {
      logSensorsBin(msg.imu, msg.baro, msg.modelInfo, msg.ekf);
    }
  }
}
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Wire.begin(33, 32);

  ServoInit();

  bool imuOk = initIMU();
  bool baroOk = initBaro();
  bool binOk = initBinLog();

  Serial.printf("[MAIN] IMU: %s  BARO: %s  BinLog: %s\n", imuOk ? "OK" : "FAIL",
                baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");

  ekf_init(&ekf, SAMPLE_RATE_MS / 1000.0f, 0.0f);
  Serial.printf("[MAIN] Target loop: %d ms (%d Hz)\n\n", SAMPLE_RATE_MS,
                1000 / SAMPLE_RATE_MS);

  // Initialize the queue to hold 100 loops worth of data (1 second buffer at
  // 100Hz)
  logQueue = xQueueCreate(100, sizeof(LogMessage));
  if (logQueue != NULL) {
    // Arduino loop() runs on Core 1 by default, so we pin logging to Core 0
    xTaskCreatePinnedToCore(sdLoggingTask,  // Task function
                            "SDLogTask",    // Task name
                            8192,           // Stack size allowed
                            NULL,           // Parameters
                            1,              // Priority (low)
                            NULL,           // Task handle
                            0               // Run on Core 0
    );
    Serial.println("[MAIN] Dual-Core SD Logging task running on Core 0");
  } else {
    Serial.println("[MAIN] ERROR: Could not create log queue!");
  }
}

// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  uint32_t loop_start = millis();
  dt = SAMPLE_RATE_MS / 1000.0f;  // Fixed dt for EKF/integration
  loop_count++;

  // ── Sensor reads ──────────────────────────────────────────────────────────
  IMUData imu = readIMU();
  BaroData baro = readBaro();

  // ── EKF ───────────────────────────────────────────────────────────────────
  gyro_yaw = imu.gyroZ;
  accel_vertical = imu.accelY - 9.81f;
  gyro_pitch = imu.gyroX;

  // Simple state machine to prevent pre-launch integration drift
  static bool is_launched = false;
  static int launch_frames = 0;

  if (!is_launched) {
    // Launch detection threshold: ~2.5G total acceleration (15 m/s^2 above
    // resting gravity)
    if (accel_vertical > 15.0f) {
      launch_frames++;
      if (launch_frames >= 5) {
        is_launched = true;
        Serial.println("[MAIN] LAUNCH DETECTED!");
      }
    } else {
      launch_frames = 0;  // Reset counter if we drop below threshold
    }

    if (!is_launched) {
      // Clamp values to prevent drift while sitting on the pad
      accel_vertical = 0.0f;
      velocity = 0.0f;
    }
  }

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

  // Package data into struct and send to Core 0 (non-blocking)
  if (logQueue != NULL) {
    LogMessage msg = {imu, baro, modelData, ekfData};
    // 0 timeout ensures if the queue fills up, the main loop drops the frame
    // rather than crashing/stalling
    xQueueSend(logQueue, &msg, 0);
  }

  // ── Periodic stats print ──────────────────────────────────────────────────
#if STATS_EVERY > 0
  if (loop_count % STATS_EVERY == 0) {
    Serial.printf("[LOOP] Heartbeat tick %d\n", loop_count);
  }
#endif

  // ── Sleep remaining budget to hit target rate ─────────────────────────────
  uint32_t elapsed = millis() - loop_start;
  if (elapsed < SAMPLE_RATE_MS) {
    delay(SAMPLE_RATE_MS - elapsed);
  }
}