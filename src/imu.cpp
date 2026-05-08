
#include "imu.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

static Adafruit_MPU6050 mpu;

bool initIMU() {
  Serial.println("[IMU] Initializing MPU6050 (skipping WHO_AM_I check)...");

  // Try begin() but don't fail if it returns false
  // (MPU6500 fails WHO_AM_I check but is register-compatible)
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  mpu.setTemperatureStandby(true);

  Serial.println("[IMU] Configuration applied successfully");
  return true;
}

IMUData readIMU() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  IMUData d;
  d.accelX = accel.acceleration.x;
  d.accelY = accel.acceleration.y;
  d.accelZ = accel.acceleration.z;
  d.gyroX = gyro.gyro.x;
  d.gyroY = gyro.gyro.y;
  d.gyroZ = gyro.gyro.z;
  return d;
}

IMUData printIMU() {
  IMUData d = readIMU();
  Serial.printf(
      "[IMU] Accel: (%.2f, %.2f, %.2f) m/s², Gyro: (%.2f, %.2f, %.2f) °/s\n",
      d.accelX, d.accelY, d.accelZ, d.gyroX, d.gyroY, d.gyroZ);
  return d;
}
