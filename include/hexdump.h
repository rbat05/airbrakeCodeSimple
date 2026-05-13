#pragma once
#include <Arduino.h>

#include "Dynamics.h"
#include "baro.h"
#include "ekf.h"
#include "imu.h"

// ── On-disk record layout (54 bytes, packed)
// ──────────────────────────────────
//
//  Offset  Size  Type    Field
//  ──────  ────  ──────  ──────────────────
//   0      4     uint32  timestamp_ms
//   4      4     float   accelX
//   8      4     float   accelY
//  12      4     float   accelZ
//  16      4     float   gyroX
//  20      4     float   gyroY
//  24      4     float   gyroZ
//  28      4     float   pressureHPa
//  32      4     float   altitudeM
//  36      4     float   filteredHeight
//  40      4     float   filteredVelocity
//  44      4     float   imuVelocityPrediction
//  48      4     float   predictedApogeeM
//  52      4     float   servoCommand
//  56      2     uint16  crc16  (covers bytes 0-55)
//  ──────  ────
//  Total: 58 bytes per record
//
// File starts with a 4-byte magic number: 0xDEAD 0xBEEF
// followed by a 2-byte record size (54) for forward-compat.
// ─────────────────────────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct BinRecord {
  uint32_t timestamp_ms;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float pressureHPa, altitudeM;
  float filteredHeight;
  float filteredVelocity;
  float imuVelocityPrediction;
  float predictedApogeeM;
  float servoCommand;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(BinRecord) == 58, "BinRecord size mismatch");

bool initBinLog();
void logSensorsBin(const IMUData& imu, const BaroData& baro,
                   const ModelData& modelData);
void flushBinLog();