#pragma once
#include <Arduino.h>

#include "baro.h"
#include "imu.h"

// ── On-disk record layout (28 bytes, packed)
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
//  28      4     float   imuTemp
//  32      4     float   baroTemp
//  36      4     float   pressureHPa
//  40      4     float   humidity
//  44      4     float   altitudeM
//  48      2     uint16  crc16  (covers bytes 0-47)
//  ──────  ────
//  Total: 50 bytes per record
//
// File starts with a 4-byte magic number: 0xDEAD 0xBEEF
// followed by a 2-byte record size (50) for forward-compat.
// ─────────────────────────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct BinRecord {
  uint32_t timestamp_ms;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float imuTemp;
  float baroTemp, pressureHPa, humidity, altitudeM;
  uint16_t crc16;
};
#pragma pack(pop)

static_assert(sizeof(BinRecord) == 50, "BinRecord size mismatch");

bool initBinLog();
void logSensorsBin(const IMUData& imu, const BaroData& baro);
void flushBinLog();