#include <Arduino.h>
#include "ServoController.h"


// ===== CONFIG =====
#define SERVO_PIN 16


#define SERVO_FREQ 50          // 50 Hz = 20 ms period
#define SERVO_CHANNEL 0
#define SERVO_RESOLUTION 16    // 16-bit PWM


// pulse limits (tune for your servo)
#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500


#define GEAR_RATIO 1.8
#define START_ANGLE 9.0




static float clampf(float x, float a, float b) {
   return (x < a) ? a : (x > b) ? b : x;
}


// Convert angle → pulse width
static uint32_t angleToPulse(float angle) {
   angle = clampf(angle, 0.0f, 180.0f);


   return (uint32_t)(
       SERVO_MIN_US +
       (angle / 180.0f) * (SERVO_MAX_US - SERVO_MIN_US)
   );
}


// ===== INIT =====
void ServoInit() {


   ledcSetup(SERVO_CHANNEL, SERVO_FREQ, SERVO_RESOLUTION);
   ledcAttachPin(SERVO_PIN, SERVO_CHANNEL);


   // safe start position
   SetServoAngle(START_ANGLE);
}


// ===== SET ANGLE =====
void SetServoAngle(float angle_deg) {


 float actuate_angle = START_ANGLE + angle_deg * GEAR_RATIO;
 uint32_t pulse = angleToPulse(angle_deg);
 SetServoPulseUs(pulse);
}


// ===== LOW LEVEL =====
void SetServoPulseUs(uint32_t pulse_us) {


   // Convert microseconds → duty cycle
   const uint32_t maxDuty = (1 << SERVO_RESOLUTION) - 1;


   uint32_t period_us = 1000000 / SERVO_FREQ; // 20000 us


   uint32_t duty = (pulse_us * maxDuty) / period_us;


   ledcWrite(SERVO_CHANNEL, duty);
}







