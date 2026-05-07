#include <Arduino.h>
#include <Wire.h>

#include "baro.h"
#include "hexdump.h"
#include "imu.h"

// Controller
#include "Dynamics.h"
#include "RocketVariables.h"

#define SAMPLE_RATE_MS 100  // 10 Hz


  // MODE
  bool fakeFlightTest = true;  // SET TO 1 WHEN LAUNCHING ACTUAL ROCKET , 0 otherwise
  bool realFlight = false;
  // Only 1 of these modes can be set to value of 1.

  
// control
float u = 0.0f;
float u_prev = 0.0f;



// params
// predictapogee dt = 0.02f; 
// main loop dt - IMPORTANT - run tests to determine this value.


void setup() {
  Serial.begin(115200);
  delay(500);  // let the monitor connect
  Serial.println("\n[MAIN] ESP32 Sensor Logger");
  Serial.println("[MAIN] Initialising I2C sensors...");

  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32

  bool imuOk = initIMU();
  bool baroOk = initBaro();
  bool binOk = initBinLog();

  // if (!imuOk || !baroOk || !binOk) {
  //   Serial.println("[MAIN] !! One or more inits failed — check wiring !!");
  //   Serial.printf("  IMU: %s  BARO: %s  BIN: %s\n", imuOk ? "OK" : "FAIL",
  //                 baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");
  //   while (true) {
  //     delay(1000);
  //   }
  // }

  Serial.printf("[MAIN] Logging binary sensor data at %d Hz\n\n",
                1000 / SAMPLE_RATE_MS);




  if (fakeFlightTest == realFlight) {
    realFlight = true;
    fakeFlightTest = false;
  }

  if (fakeFlightTest == 1) {
    state.h = 120.0;
    state.v = 105.3;
  }
             
}

void loop() {
  IMUData imu = readIMU();
  BaroData baro = readBaro();

  logSensorsBin(imu, baro);
  delay(SAMPLE_RATE_MS);



  // Run fake test
  if (fakeFlightTest == 1) {

   
      u = OptimiseControlInputBinarySearchConstraint(state.h, state.v,u_prev,0);

      float h_pred = PredictApogee(state.h, state.v, u);

      // ---- actuator ----
      // set_servo(u);



  }

}
