#include "hexdump.h"

#include <FS.h>
#include <SD.h>  // ESP32 built-in — do NOT add SD to lib_deps
#include <SPI.h>

// ── Pin config
// ────────────────────────────────────────────────────────────────
#define SD_MOSI 23
#define SD_MISO 19
#define SD_SCLK 18
#define SD_CS 5

// ── Buffer config
// ───────────────────────────────────────────────────────────── 100 records ×
// 50 bytes = 5 000 bytes per flush. SD cards prefer writes in multiples of the
// sector size (512 B). 5 000 B = ~9.7 sectors — close enough; tweak
// BUFFER_RECORDS to taste.
#define BUFFER_RECORDS 100

// File header
static const uint8_t FILE_MAGIC[4] = {0xDE, 0xAD, 0xBE, 0xEF};
static const uint16_t RECORD_SIZE = sizeof(BinRecord);  // 50
static const char* BIN_FILE_PREFIX = "/datalog_";
static const char* BIN_FILE_SUFFIX = ".bin";
static char s_logFile[24] = {0};

// ── Internal state
// ────────────────────────────────────────────────────────────
static BinRecord s_buf[BUFFER_RECORDS];
static uint16_t s_count = 0;
static bool s_ready = false;

// ── CRC-16/CCITT-FALSE
// ────────────────────────────────────────────────────────
static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

// ── Helpers
// ───────────────────────────────────────────────────────────────────
static bool flushBuffer() {
  File f = SD.open(s_logFile, FILE_APPEND);
  if (!f) {
    Serial.println("[BIN] Open failed");
    return false;
  }
  // Write the whole buffer in one call — single large block transfer
  size_t toWrite = s_count * sizeof(BinRecord);
  size_t written = f.write(reinterpret_cast<const uint8_t*>(s_buf), toWrite);
  f.close();

  if (written != toWrite) {
    Serial.printf("[BIN] Partial write: %u / %u bytes\n", written, toWrite);
    return false;
  }
  Serial.printf("[BIN] Flushed %u records (%u bytes)\n", s_count, written);
  s_count = 0;
  return true;
}

static bool pickNextLogFileName() {
  for (uint16_t index = 0; index <= 999; index++) {
    snprintf(s_logFile, sizeof(s_logFile), "%s%03u%s", BIN_FILE_PREFIX, index,
             BIN_FILE_SUFFIX);
    if (!SD.exists(s_logFile)) {
      return true;
    }
  }

  s_logFile[0] = '\0';
  return false;
}

// ── Public API
// ────────────────────────────────────────────────────────────────
bool initBinLog() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("[BIN] SD mount failed");
    return false;
  }

  if (!pickNextLogFileName()) {
    Serial.println("[BIN] No free log filename available");
    return false;
  }

  File f = SD.open(s_logFile, FILE_WRITE);
  if (!f) {
    Serial.println("[BIN] Cannot create log file");
    return false;
  }
  f.write(FILE_MAGIC, 4);
  f.write(reinterpret_cast<const uint8_t*>(&RECORD_SIZE), 2);
  f.close();
  Serial.printf("[BIN] Created %s (record size: %u bytes)\n", s_logFile,
                RECORD_SIZE);

  s_ready = true;
  return true;
}

void logSensorsBin(const IMUData& imu, const BaroData& baro) {
  if (!s_ready) return;

  BinRecord& r = s_buf[s_count];
  r.timestamp_ms = millis();
  r.accelX = imu.accelX;
  r.accelY = imu.accelY;
  r.accelZ = imu.accelZ;
  r.gyroX = imu.gyroX;
  r.gyroY = imu.gyroY;
  r.gyroZ = imu.gyroZ;
  r.imuTemp = imu.tempC;
  r.baroTemp = baro.tempC;
  r.pressureHPa = baro.pressureHPa;
  r.humidity = baro.humidity;
  r.altitudeM = baro.altitudeM;

  // CRC covers everything except the crc16 field itself
  r.crc16 = crc16(reinterpret_cast<const uint8_t*>(&r),
                  sizeof(BinRecord) - sizeof(uint16_t));
  s_count++;

  if (s_count >= BUFFER_RECORDS) flushBuffer();
}

void flushBinLog() {
  if (s_ready && s_count > 0) {
    Serial.printf("[BIN] Force-flushing %u records\n", s_count);
    flushBuffer();
  }
}