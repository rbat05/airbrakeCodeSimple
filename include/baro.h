#pragma once
#include <Arduino.h>

struct BaroData {
  float tempC;        // °C
  float pressureHPa;  // hPa
  float humidity;     // %RH
  float altitudeM;    // metres (approx, sea-level baseline)
};

bool initBaro();
BaroData readBaro();
