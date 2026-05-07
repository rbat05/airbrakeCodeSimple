#include <Arduino.h>
#include <Wire.h>

#include "baro.h"
#include "hexdump.h"
#include "imu.h"

#define SAMPLE_RATE_MS 100  // 10 Hz

void setup() {
  Serial.begin(115200);
  delay(500);  // let the monitor connect
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Serial.println("[MAIN] Initialising I2C sensors...");

  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32

  bool imuOk = initIMU();
  bool baroOk = initBaro();
  bool binOk = initBinLog();

  // if (!imuOk || !baroOk || !binOk) {
  //   Serial.println("[MAIN] !! One or more inits failed — check wiring !!");
  //   Serial.printf("  IMU: %s  BARO: %s  BIN: %s\n", imuOk ? "OK" : "FAIL",
  //                 baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");
  //   while (true) {
  //     delay(1000);
  //   }
  // }

  Serial.printf("[MAIN] Logging binary sensor data at %d Hz\n\n",
                1000 / SAMPLE_RATE_MS);
}

void loop() {
  IMUData imu = readIMU();
  BaroData baro = readBaro();

  logSensorsBin(imu, baro);
  delay(SAMPLE_RATE_MS);
}
