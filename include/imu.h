#pragma once
#include <Arduino.h>

struct IMUData {
  float accelX, accelY, accelZ;  // m/s²
  float gyroX, gyroY, gyroZ;     // °/s
};

bool initIMU();
IMUData readIMU();
