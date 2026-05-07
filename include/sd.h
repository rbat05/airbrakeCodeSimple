#pragma once
#include <Arduino.h>

#include "baro.h"
#include "imu.h"

bool initSD();
void logSensors(const IMUData& imu, const BaroData& baro);
void flushLog();  // call on shutdown / critical error to force-write buffer
