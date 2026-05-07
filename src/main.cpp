#include <Arduino.h>
#include <Wire.h>

#include "baro.h"
#include "hexdump.h"
#include "imu.h"
#include "sd.h"
#include "ekf.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// ── Test config
#define SAMPLE_RATE_MS 100      // 10 Hz
#define TEST_DURATION_MS 30000  // run for 30 s then stop and report

// ── State
static uint32_t s_sampleCount = 0;
static uint32_t s_startMs = 0;
static bool s_done = false;

// Timing accumulators for benchmarking
static uint32_t s_csvTotalUs = 0;
static uint32_t s_binTotalUs = 0;

// LED Pin
static const uint8_t LED_PIN = 2;

// ── Helpers
static void printSensors(const IMUData& imu, const BaroData& baro) {
  Serial.printf(
      "[%6lu ms] "
      "Accel: %6.3f %6.3f %6.3f m/s²  "
      "Gyro: %6.2f %6.2f %6.2f °/s  "
      "imuT: %.1f°C  "
      "baroT: %.1f°C  %.1f hPa  %.1f%%  %.1fm\n",
      millis(), imu.accelX, imu.accelY, imu.accelZ, imu.gyroX, imu.gyroY,
      imu.gyroZ, imu.tempC, baro.tempC, baro.pressureHPa, baro.humidity,
      baro.altitudeM);
}

static void printReport() {
  Serial.println("\n════════════════════════════════════════");
  Serial.println("           LOGGER BENCHMARK REPORT");
  Serial.println("════════════════════════════════════════");
  Serial.printf("  Samples collected : %lu\n", s_sampleCount);
  Serial.printf("  Duration          : %.1f s\n", TEST_DURATION_MS / 1000.0f);
  Serial.printf("  Sample rate       : %d Hz\n", 1000 / SAMPLE_RATE_MS);
  Serial.println("----------------------------------------");
  if (s_sampleCount > 0) {
    float csvAvgUs = (float)s_csvTotalUs / s_sampleCount;
    float binAvgUs = (float)s_binTotalUs / s_sampleCount;
    Serial.printf("  CSV avg per call  : %.1f µs\n", csvAvgUs);
    Serial.printf("  BIN avg per call  : %.1f µs\n", binAvgUs);
    Serial.printf("  Speedup (bin/csv) : %.2fx faster\n",
                  csvAvgUs / max(binAvgUs, 0.01f));
  }
  Serial.println("════════════════════════════════════════");
  Serial.println("  Buffers flushed to SD. Check:");
  Serial.println("    /datalog.csv  — open in Excel / any editor");
  Serial.println("    /datalog.bin  — decode with decode_log.py");
  Serial.println("════════════════════════════════════════\n");
}

// ── Arduino entry points

BaroData baro;
IMUData imu;
EKF ekf;
float dt;
float velocity = 0.0f;


void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32-Sensor-Logger"); // Bluetooth device name
  delay(2000);  // let the monitor connect
  // Serial.println("\n[MAIN] ESP32 Sensor Logger Test");
  // Serial.println("[MAIN] Initialising I2C sensors...");
  // SerialBT.println("\n[MAIN] ESP32 Bluetooth Started");
  //delay(1000);  // let Bluetooth initialize 
  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32




  bool imuOk = initIMU();
  bool baroOk = initBaro();


  baro = readBaro();
  float h0 = baro.altitudeM;  // use first baro reading as height reference for EKF

  // Serial.println("[MAIN] Initialising SD loggers...");
  // bool csvOk = initSD();
  // bool binOk = initBinLog();

  // if (!imuOk || !baroOk || !csvOk || !binOk) {
  //   Serial.println("[MAIN] !! One or more inits failed — check wiring !!");
  //   Serial.printf("  IMU: %s  BARO: %s  CSV-SD: %s  BIN-SD: %s\n",
  //                 imuOk ? "OK" : "FAIL", baroOk ? "OK" : "FAIL",
  //                 csvOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");
  //   // Halt — blink LED as error indicator
  //   pinMode(LED_PIN, OUTPUT);
  //   while (true) {
  //     digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  //     delay(200);
  //   }
  // }
  dt= 0.1f;   // 100Hz IMU rate (Can control to be loop rate)
  ekf_init(&ekf, dt, h0);
  // Serial.printf("[MAIN] All systems go — logging for %lu s at %d Hz\n\n",
  //               TEST_DURATION_MS / 1000UL, 1000 / SAMPLE_RATE_MS);
  // s_startMs = millis();
}

void loop() {
  // Print imu and baro data to serial

  imu = readIMU();
  baro = readBaro();
  //printSensors(imu, baro);
  static uint32_t prev_ms = millis();
  uint32_t now = millis();
  

  dt = (now - prev_ms) / 1000.0f;
  prev_ms = now;
 // print all three accelerations and gyros to serial
  //float accel_z = imu.accelZ;    // m/s^2
  //float gyro_y = imu.gyroY;     // rad/s
  float gyro_yaw = imu.gyroZ;     // rad/s
  float accel_vertical = imu.accelY - 9.81f;    // m/s^2
  // float accel_x = imu.accelX;    // m/s^2
  float gyro_pitch = imu.gyroX;     // rad/s
  float barometer_raw = baro.altitudeM + 42;   // metres

  // serial print accel and gyro value for pitch, yaw and vertical accel
  //Serial.printf("\nAccel vertical: %.2f m/s^2  Gyro pitch: %.2f °/s  Gyro yaw: %.2f °/s\n", accel_vertical, gyro_pitch, gyro_yaw);
  //SerialBT.printf("\nAccel vertical
   //velocity = velocity + accel_vertical * dt;  // simple velocity estimate by integrating accel (for comparison with EKF)
   // simple velocity estimate by integrating accel (for comparison with EKF)
  //Serial.printf("\nHeight before EKF : %.2f m  Velocity: %.2f m/s\n", barometer_raw, velocity);
  //SerialBT.printf("\nHeight before EKF : %.2f m  Velocity: %.2f m/s\n", barometer_raw, velocity);
  //delay(100);
  ekf_predict(&ekf, accel_vertical, gyro_pitch, gyro_yaw);
  // float barometer_raw = baro.altitudeM;   // metres
  ekf_update(&ekf, barometer_raw);

  //Serial.printf(
  //  "\nHeight after EKF : %.2f m  Velocity: %.2f m/s\n",
  //  ekf.x[0],
  //  ekf.x[1] );


  Serial.printf(
    "%lu,%.3f,%.3f,%.3f,%.3f\n",
    millis(),
    barometer_raw,
    velocity,
    ekf.x[0],
    ekf.x[1]
  );
  delay(100);

  // if (s_done) return;

  // // ── Check test window
  // ───────────────────────────────────────────────────── if (millis() -
  // s_startMs >= TEST_DURATION_MS) {
  //   flushLog();     // force-flush remaining CSV rows
  //   flushBinLog();  // force-flush remaining binary records
  //   printReport();
  //   s_done = true;
  //   return;
  // }

  // // ── Read sensors
  // ────────────────────────────────────────────────────────── IMUData imu =
  // readIMU(); BaroData baro = readBaro();

  // // Print every 10th sample to serial (avoid flooding at 10 Hz)
  // if (s_sampleCount % 10 == 0) printSensors(imu, baro);

  // // ── Benchmark CSV logger
  // ────────────────────────────────────────────────── uint32_t t0 = micros();
  // logSensors(imu, baro);
  // s_csvTotalUs += micros() - t0;

  // // ── Benchmark binary logger
  // ─────────────────────────────────────────────── t0 = micros();
  // logSensorsBin(imu, baro);
  // s_binTotalUs += micros() - t0;

  // s_sampleCount++;
  // delay(SAMPLE_RATE_MS);

}
