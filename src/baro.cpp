#include "baro.h"

#include <Adafruit_BME280.h>
#include <Wire.h>

// Reference altitude:
// Get some pressure readings from the sensor, and set to REFERENCE_PRESSURE_HPA
// Set REFERENCE_ALTITUDE_M to 0m
// This makes it so that the altitude readings are relative to the launch site
// i.e Above Ground Level (AGL) instead of Mean Sea Level (MSL).
#define REFERENCE_ALTITUDE_M 0.0f
#define REFERENCE_PRESSURE_HPA 1030.4f

static Adafruit_BME280 bme;

// This will be calculated from the launch-day calibration and used for altitude
// readings. Do not change it.
static float s_seaLevelHPa = 0.0f;

bool initBaro() {
  if (!bme.begin(0x76)) {  // try 0x76; change to 0x77 if SDO pulled high
    Serial.println("[BARO] BME280 not found");
    return false;
  }

  s_seaLevelHPa =
      bme.seaLevelForAltitude(REFERENCE_ALTITUDE_M, REFERENCE_PRESSURE_HPA);
  Serial.printf(
      "[BARO] Reference calibration: altitude=%.1f m, pressure=%.2f hPa\n",
      REFERENCE_ALTITUDE_M, REFERENCE_PRESSURE_HPA);
  Serial.printf("[BARO] Derived sea-level pressure: %.2f hPa\n", s_seaLevelHPa);

  // Indoor navigation mode — good balance of speed vs noise
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,    // temperature
                  Adafruit_BME280::SAMPLING_X16,   // pressure
                  Adafruit_BME280::SAMPLING_NONE,  // humidity disabled
                  Adafruit_BME280::FILTER_X16, Adafruit_BME280::STANDBY_MS_0_5);
  Serial.println("[BARO] BME280 initialised");
  return true;
}

BaroData readBaro() {
  BaroData d;
  d.pressureHPa = bme.readPressure() / 100.0f;
  d.altitudeM = bme.readAltitude(s_seaLevelHPa);
  return d;
}

BaroData printBaro() {
  BaroData d = readBaro();
  Serial.printf("[BARO] Pressure: %.2f hPa, Altitude: %.2f m\n", d.pressureHPa,
                d.altitudeM);
  return d;
}
