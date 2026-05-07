#pragma once
#include <Arduino.h>

struct BaroData {
  float pressureHPa;  // hPa
  float altitudeM;    // metres (approx, sea-level baseline)
};

bool initBaro();
BaroData readBaro();
BaroData printBaro();
