#pragma once
#include "baro.h"
#include "imu.h"

void initHITL();
void updateHITL();
BaroData readBaroHITL();
IMUData readIMUHITL();
