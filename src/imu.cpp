
#include "imu.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

static Adafruit_MPU6050 mpu;

bool initIMU() {
  if (!mpu.begin()) {
    Serial.println("[IMU] MPU6050 not found");
    return false;
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[IMU] MPU6050 initialised");
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
  d.tempC = temp.temperature;
  return d;
}
