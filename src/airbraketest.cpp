
// #include <Arduino.h>
// #include "ServoController.h"


// #include <Wire.h>
// #include <stdlib.h>

// #include "baro.h"
// #include "hexdump.h"
// #include "imu.h"

// // Controller
// #include "Dynamics.h"
// #include "RocketVariables.h"
// #include "ServoController.h"

// #include "ekf.h"

// // EKF ekf;   // global — must persist between loop() calls


// // control
// float u = 0.0f;
// float u_prev = 0.0f;

// // timer
// hw_timer_t* timer = NULL;

// volatile bool controlFlag = false;


// void IRAM_ATTR onTimer() {
//    controlFlag = true;
// }


// void initSim(void) {

//   Serial.begin(115200);
//   delay(500);  // let the monitor connect
//   // Serial.println("\n[MAIN] ESP32 Sensor Logger");
//   // Serial.println("[MAIN] Initialising I2C sensors...");
//   // Wire.begin(33, 32);  // SDA=GPIO33, SCL=GPIO32
//   // bool imuOk = initIMU();
//   // bool baroOk = initBaro();
//   // bool binOk = initBinLog();

//   state.h = 120.0;
//   state.v = 105.3;

//   // ServoInit();

//   // for (int i = 0; i < 2; i++) {
//   //   SetServoAngle(0);
//   //   delay(1000);
//   //   SetServoAngle(70);
//   //   delay(1000);
//   // }

//   // SetServoAngle(0); 
//   // delay(3000);

// //   float dt = 0.01f;   // 100Hz IMU rate (Can control to be loop rate)
// //   ekf_init(&ekf, dt, h0);

//   // void ekf_predict(EKF *ekf, float a_body, float w_pitch, float w_yaw) {
//   // void ekf_update(EKF *ekf, float z_baro) {

// // float ekf_get_height(EKF *ekf)    { return ekf->x[0]; }
// // float ekf_get_velocity(EKF *ekf)  { return ekf->x[1]; }


//   // timer setup
//   timer = timerBegin(0, 80, true);
//   timerAttachInterrupt(timer, &onTimer, true);
//   timerAlarmWrite(timer, 50000, true);  // 50000 us = 50 Hz
//   timerAlarmEnable(timer);
//   delay(1000); // wait 3 seconds
// }

// void RunAirbrakeSim(void) {
//   initSim();
//   int iteration = 0;

//   // Fake flight test
//   while (1) {
//     if (ApogeeDetected() == 0) {
//       if (!controlFlag) {
//         continue;
//       }
//       controlFlag = false;
//       // ---- simulate dynamics ----
//       rk4(&state.h, &state.v, 0.0f, GetCd(u), m_final, 0.05);  // replace with sensor add some point
//       float h_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;
//       float v_noise = ((float)rand() / RAND_MAX) * 2.0f - 1.0f;

//       // state.h += h_noise;
//       // state.v += v_noise;

//       u = OptimiseControlInputBinarySearchConstraint(state.h, state.v,u_prev,iteration);
//       // SetServoAngle(u);   // u = 0–180 degrees
//       u_prev = u;
//       float h_pred = PredictApogee(state.h, state.v, u);

//       // set_servo(u);
//       Serial.printf("h=%.3f, v=%.2f, u=%.2f, predicted apogee=%.2f, iteration: %d\n", state.h, state.v, u,h_pred,iteration);
//       iteration++;
//     }
//   }
// }





