#include "sd.h"

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
// ───────────────────────────────────────────────────────────── Each CSV row is
// ~80 chars. 64 rows ≈ 5 KB — well within ESP32 heap. Increase BUFFER_ROWS for
// fewer writes; decrease if RAM is tight.
#define BUFFER_ROWS 64
#define ROW_MAX_LEN 96

static const char* LOG_FILE = "/datalog.csv";
static const char* CSV_HEADER =
    "millis,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,imuTemp,"
    "baroTemp,pressureHPa,humidity,altitudeM\n";

// ── Internal state
// ────────────────────────────────────────────────────────────
static char s_buf[BUFFER_ROWS][ROW_MAX_LEN];
static uint8_t s_count = 0;
static bool s_ready = false;

// ── Helpers
// ───────────────────────────────────────────────────────────────────
static bool writeBufferToSD() {
  File f = SD.open(LOG_FILE, FILE_APPEND);
  if (!f) {
    Serial.println("[SD] Failed to open file for append");
    return false;
  }
  for (uint8_t i = 0; i < s_count; i++) {
    f.print(s_buf[i]);
  }
  f.close();
  s_count = 0;
  return true;
}

// ── Public API
// ────────────────────────────────────────────────────────────────
bool initSD() {
  SPI.begin(SD_SCLK, SD_MISO, SD_MOSI, SD_CS);

  if (!SD.begin(SD_CS)) {
    Serial.println("[SD] Mount failed — check wiring/card");
    return false;
  }

  // Write header only if the file doesn't exist yet
  if (!SD.exists(LOG_FILE)) {
    File f = SD.open(LOG_FILE, FILE_WRITE);
    if (!f) {
      Serial.println("[SD] Could not create log file");
      return false;
    }
    f.print(CSV_HEADER);
    f.close();
  }

  s_ready = true;
  Serial.printf("[SD] Ready — logging to %s\n", LOG_FILE);
  return true;
}

void logSensors(const IMUData& imu, const BaroData& baro) {
  if (!s_ready) return;

  // Format one CSV row into the next buffer slot
  snprintf(s_buf[s_count], ROW_MAX_LEN,
           "%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
           millis(), imu.accelX, imu.accelY, imu.accelZ, imu.gyroX, imu.gyroY,
           imu.gyroZ, imu.tempC, baro.tempC, baro.pressureHPa, baro.humidity,
           baro.altitudeM);
  s_count++;

  // Flush when buffer is full
  if (s_count >= BUFFER_ROWS) {
    if (!writeBufferToSD()) {
      Serial.println("[SD] Write error — buffer dropped");
      s_count = 0;  // don't stall; keep collecting
    }
  }
}

void flushLog() {
  if (!s_ready || s_count == 0) return;
  Serial.printf("[SD] Flushing %u remaining rows\n", s_count);
  writeBufferToSD();
}