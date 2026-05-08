// #include <Arduino.h>
// #include <Wire.h>
// #include <stdlib.h>




// #include "baro.h"
// #include "hexdump.h"
// #include "imu.h"


// // Controller
// #include "Dynamics.h"
// #include "RocketVariables.h"
// #include "ServoController.h"


// #define SAMPLE_RATE_MS 10  // 100 Hz


// // MODE
// bool fakeFlightTest = true;  // SET TO 1 WHEN LAUNCHING ACTUAL ROCKET , 0 otherwise
// bool realFlight = false;
// // Only 1 of these modes can be set to value of 1.


// // control
// float u = 0.0f;
// float u_prev = 0.0f;


// // timer
// hw_timer_t* timer = NULL;


// volatile bool controlFlag = false;


// void IRAM_ATTR onTimer() {
//    controlFlag = true;
// }


// void setup() {
//  Serial.begin(115200);
//  delay(500);  // let the monitor connect
//  Serial.println("\n[MAIN] ESP32 Sensor Logger");
//  Serial.println("[MAIN] Initialising I2C sensors...");


//  Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32


//  bool imuOk = initIMU();
//  bool baroOk = initBaro();
//  bool binOk = initBinLog();


//  // if (!imuOk || !baroOk || !binOk) {
//  //   Serial.println("[MAIN] !! One or more inits failed — check wiring !!");
//  //   Serial.printf("  IMU: %s  BARO: %s  BIN: %s\n", imuOk ? "OK" : "FAIL",
//  //                 baroOk ? "OK" : "FAIL", binOk ? "OK" : "FAIL");
//  //   while (true) {
//  //     delay(1000);
//  //   }
//  // }


//  Serial.printf("[MAIN] Logging binary sensor data at %d Hz\n\n",
//                1000 / SAMPLE_RATE_MS);


//  if (fakeFlightTest == realFlight) {
//    realFlight = true;
//    fakeFlightTest = false;
//  }


//  if (fakeFlightTest == 1) {
//    state.h = 120.0;
//    state.v = 105.3;
//  }


//  ServoInit();




//  for (int i = 0; i < 2; i++) {
//    SetServoAngle(0);
//    delay(1000);


//    SetServoAngle(70);
//    delay(1000);
//  }


//    SetServoAngle(0);
//      delay(3000);


//  //   while (1) {
//  //     SetServoAngle(0);
//  //     delay(1000);


//  //     SetServoAngle(70);
//  //     delay(1000);
//  // }


//  // timer setup
//  timer = timerBegin(0, 80, true);


//  timerAttachInterrupt(timer, &onTimer, true);


//  // timerAlarmWrite(timer, 10000, true);
//  // // 10000 us = 100 Hz


//  timerAlarmWrite(timer, 50000, true);
//  // 50000 us = 50 Hz


//  timerAlarmEnable(timer);
//  delay(5000); // wait 3 seconds


// }


// int iteration = 0;


// void loop() {


//  // Real flight
//  if (realFlight == 1) {
//    IMUData imu = readIMU();
//    BaroData baro = readBaro();
//    logSensorsBin(imu, baro);
//    delay(SAMPLE_RATE_MS);








//  // Fake flight test
//  } else if (fakeFlightTest == 1) {
//    if (ApogeeDetected() == 0) {
//      if (!controlFlag) {
//        return;
//      }
//      controlFlag = false;
//      // ---- simulate dynamics ----
//      rk4(&state.h, &state.v, 0.0f, GetCd(u), m_final, 0.05);  // replace with sensor add some point
//      float h_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
//      float v_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;


//      state.h += h_noise;
//      state.v += v_noise;


//      u = OptimiseControlInputBinarySearchConstraint(state.h, state.v,u_prev,iteration);
//      SetServoAngle(u);   // u = 0–180 degrees
//      u_prev = u;
//      float h_pred = PredictApogee(state.h, state.v, u);


//      // set_servo(u);
//      Serial.printf("h=%.3f, v=%.2f, u=%.2f, predicted apogee=%.2f, iteration: %d\n", state.h, state.v, u,h_pred,iteration);
//      iteration++;
//    }
//  }
// }





