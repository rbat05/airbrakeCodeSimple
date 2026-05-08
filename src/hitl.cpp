#include "hitl.h"

#include <Arduino.h>
#include <math.h>

static uint32_t s_lastUpdateUs = 0;
static bool s_launched = false;

static float s_t = 0.0f;  // Time since launch (s)
static float s_y = 0.0f;  // Altitude (m)
static float s_v = 0.0f;  // Velocity (m/s)
static float s_a = 0.0f;  // Net acceleration (m/s^2)

// Rocket Specifications
const float MASS_INITIAL = 2.8f;   // kg
const float MASS_FINAL = 2.45f;    // kg
const float BURN_TIME = 1.7f;      // s
const float THRUST_AVG = 200.0f;   // N
const float GRAVITY = 9.81f;       // m/s^2
const float AIR_DENSITY = 1.225f;  // kg/m^3

// Drag coefficient * Reference Area. Tuned to hit ~450m apogee.
// Apogee w/o drag ~ 744m. This value brings it down to ~450m.
const float CDA = 0.0035f;

void initHITL() {
  s_lastUpdateUs = micros();
  s_launched = false;
  s_t = 0.0f;
  s_y = 0.0f;
  s_v = 0.0f;
  s_a = 0.0f;
  Serial.println("[HITL] Simulator Initialized");
}

void updateHITL() {
  uint32_t now = micros();
  float dt = (now - s_lastUpdateUs) / 1000000.0f;
  s_lastUpdateUs = now;

  // Auto-launch after 5 seconds to simulate pre-launch pad time
  if (!s_launched && millis() > 5000) {
    s_launched = true;
    Serial.println("[HITL] Auto-launching simulator!");
    dt = 0.0f;  // Reset dt on launch to avoid large jump
  }

  if (dt <= 0.0f || !s_launched) {
    s_a = 0.0f;  // On pad, physical acceleration is 0
    return;
  }

  s_t += dt;

  float mass = MASS_INITIAL;
  float thrust = 0.0f;

  // Motor burn phase
  if (s_t <= BURN_TIME) {
    thrust = THRUST_AVG;
    // Linear mass interpolation
    mass = MASS_INITIAL - ((MASS_INITIAL - MASS_FINAL) / BURN_TIME) * s_t;
  } else {
    mass = MASS_FINAL;  // Coast phase
  }

  // Drag force = 1/2 * rho * v^2 * CdA
  float drag = 0.5f * AIR_DENSITY * s_v * s_v * CDA;
  if (s_v < 0) {
    drag = -drag;  // Drag opposes motion
  }

  // Net force (upwards is positive)
  float net_force = thrust - (mass * GRAVITY) - drag;

  // Acceleration
  s_a = net_force / mass;

  // Update velocity and position
  if (s_y >= 0.0f || s_a > 0.0f) {
    s_v += s_a * dt;
    s_y += s_v * dt;
  }

  // Ground collision / resting on pad
  if (s_y < 0.0f) {
    s_y = 0.0f;
    s_v = 0.0f;
    s_a = 0.0f;
  }
}

BaroData readBaroHITL() {
  BaroData d;

  // Sea level pressure from baro.cpp reference
  const float P0 = 1018.3f;

  // Barometric formula
  d.pressureHPa =
      P0 * exp(-GRAVITY * 0.0289644f * s_y / (8.3144598f * 288.15f));
  d.altitudeM = s_y;

  // Add some random noise (+/- 0.5m)
  d.altitudeM += ((rand() % 100) / 100.0f - 0.5f) * 1.0f;
  return d;
}

IMUData readIMUHITL() {
  IMUData d;
  d.accelX = 0.0f;
  d.accelZ = 0.0f;

  // IMU Y measures Proper Acceleration.
  // On pad: net accel = 0, but proper accel = +9.81 (resisting gravity)
  // In flight: proper accel = net_accel + 9.81
  d.accelY = s_a + GRAVITY;

  // Add some random noise (+/- 0.25 m/s^2)
  d.accelY += ((rand() % 100) / 100.0f - 0.5f) * 0.5f;

  d.gyroX = 0.0f;
  d.gyroY = 0.0f;
  d.gyroZ = 0.0f;
  return d;
}
