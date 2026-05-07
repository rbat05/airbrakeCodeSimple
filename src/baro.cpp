#include "baro.h"

#include <Adafruit_BME280.h>
#include <Wire.h>

#define SEA_LEVEL_HPA 1013.25f

static Adafruit_BME280 bme;

bool initBaro() {
  if (!bme.begin(0x76)) {  // try 0x76; change to 0x77 if SDO pulled high
    Serial.println("[BARO] BME280 not found");
    return false;
  }
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
  d.altitudeM = bme.readAltitude(SEA_LEVEL_HPA);
  return d;
}
