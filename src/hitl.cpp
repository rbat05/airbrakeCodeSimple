#include "hitl.h"

#include <Arduino.h>

#include "hitl_data.h"

static uint32_t s_currentDataIndex = 0;
static bool s_simFinished = false;

void initHITL() {
  s_currentDataIndex = 0;
  s_simFinished = false;
  Serial.println("[HITL] Data playback Simulator Initialized");
  Serial.printf("[HITL] Loaded %d frames of real telemetry\n", HITL_DATA_SIZE);
}

void updateHITL() {
  if (s_currentDataIndex < HITL_DATA_SIZE - 1) {
    s_currentDataIndex++;
  } else {
    if (!s_simFinished) {
      s_simFinished = true;
      Serial.println("[HITL] Data playback finished!");
    }
  }
}

BaroData readBaroHITL() {
  BaroData d;
  if (s_currentDataIndex < HITL_DATA_SIZE) {
    d.pressureHPa = hitl_data[s_currentDataIndex].pressureHPa;
    d.altitudeM = hitl_data[s_currentDataIndex].altitudeM;
  }
  return d;
}

IMUData readIMUHITL() {
  IMUData d;
  if (s_currentDataIndex < HITL_DATA_SIZE) {
    d.accelX = hitl_data[s_currentDataIndex].accelX;
    d.accelY = hitl_data[s_currentDataIndex].accelY;
    d.accelZ = hitl_data[s_currentDataIndex].accelZ;
    d.gyroX = hitl_data[s_currentDataIndex].gyroX;
    d.gyroY = hitl_data[s_currentDataIndex].gyroY;
    d.gyroZ = hitl_data[s_currentDataIndex].gyroZ;
  }
  return d;
}
