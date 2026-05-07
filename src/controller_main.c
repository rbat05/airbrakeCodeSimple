// #include <math.h>
// #include <stdio.h>

// #include "driver/ledc.h"
// #include "esp_timer.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// // #include "DynamicsNew.h"
// #include "RocketVariables.h"

// // ---- globals ----
// volatile int control_flag = 0;

// // control
// float u = 0.0f;
// float u_prev = 0.0f;

// // params
// // float dt = 0.02f; // 50 Hz
// // float dt = 0.015f; // 20 Hz

// // sim stuff
// // inject h = 100m and v = 150m/s when coast detected  - NOT FOR REALTIME USE
// int sim_status = 1;  // INJECTING SENSOR DATA - NOT FOR REAL FLIGHT USE

// float h_coast_start_sim = 120.0f;
// float v_coast_start_sim = 105.3f;

// // logging
// #define MAX_SIZE 1000
// int iteration = 0;
// float altitude_history[MAX_SIZE];
// float velocity_history[MAX_SIZE];
// float control_input_history[MAX_SIZE];
// float predicted_apogee_history[MAX_SIZE];
// float optimise_time_history[MAX_SIZE];
// float loop_time_history[MAX_SIZE];

// float time_history[MAX_SIZE];

// // ---- TIMER ISR ----
// void IRAM_ATTR timer_isr(void* arg) { control_flag = 1; }

// void setup_timer() {
//   const esp_timer_create_args_t timer_args = {.callback = &timer_isr,
//                                               .name = "control_timer"};

//   esp_timer_handle_t timer;
//   esp_timer_create(&timer_args, &timer);

//   // 20 ms = 50 Hz
//   // esp_timer_start_periodic(timer, 20000);

//   esp_timer_start_periodic(timer, loop_dt * 1000000.0);
// }

// // void setup_servo() {

// //     ledc_timer_config_t timer = {
// //         .speed_mode = LEDC_LOW_SPEED_MODE,
// //         .timer_num = LEDC_TIMER_0,
// //         .duty_resolution = LEDC_TIMER_16_BIT,
// //         .freq_hz = 50
// //     };
// //     ledc_timer_config(&timer);

// //     ledc_channel_config_t channel = {
// //         .gpio_num = 18,
// //         .speed_mode = LEDC_LOW_SPEED_MODE,
// //         .channel = LEDC_CHANNEL_0,
// //         .timer_sel = LEDC_TIMER_0,
// //         .duty = 0
// //     };
// //     ledc_channel_config(&channel);
// // }

// // void set_servo(float angle) {

// //     // clamp
// //     if (angle < 0.0f) angle = 0.0f;
// //     if (angle > 70.0f) angle = 70.0f;

// //     // map to pulse (~1ms–2ms)
// //     float duty = 3277 + (angle / 70.0f) * 3277;

// //     ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)duty);
// //     ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
// // }

// void app_main(void) {
//   if (sim_status == 1) {
//     state.h = h_coast_start_sim;
//     state.v = v_coast_start_sim;
//   } else {
//     state.h = 0.0;
//     state.v = 0.0;
//   }

//   printf("Phase 1: MPC test starting\n");

//   setup_timer();
//   // setup_servo();

//   vTaskDelay(pdMS_TO_TICKS(10000));

//   // need to trigger this on launch interrupt
//   int launch_time = esp_timer_get_time();
//   int start_time = 0;
//   int end_time = 0;

//   float h_pred_initial = PredictApogee(state.h, state.v, 0);

//   while (1) {
//     if (control_flag) {
//       int loop_start_time = esp_timer_get_time();

//       control_flag = 0;

//       if (ApogeeDetected()) {
//         break;
//       }

//       // ---- simulate dynamics ----
//       rk4(&state.h, &state.v, 0.0f, GetCd(u), m_final,
//           loop_dt);  // replace with sensor add some point

//       // ---- run MPC ----
//       start_time = esp_timer_get_time();

//       // u = OptimiseControlInput(state.h, state.v, u_prev);
//       // u = OptimiseControlInputBinarySearch(state.h, state.v, u_prev);

//       u = OptimiseControlInputBinarySearchConstraint(state.h, state.v,
//       u_prev,
//                                                      iteration);

//       end_time = esp_timer_get_time();

//       float h_pred = PredictApogee(state.h, state.v, u);

//       // ---- actuator ----
//       // set_servo(u);

//       u_prev = u;

//       // ---- debug ----
//       // if (iteration == 0 || iteration == 1 || iteration == 2 || iteration
//       ==
//       // 3|| iteration == 4|| iteration == 5|| iteration == 6|| iteration ==
//       7||
//       // iteration == 8|| iteration == 9|| iteration == 10|| iteration == 11)
//       {

//       //  printf("h=%.3f v=%.2f u=%.2f dt_ms=%.3f\n", state.h, state.v, u,
//       //  (float)(end_time - start_time)/1000.0f);
//       // }
//       int loop_end_time = esp_timer_get_time();

//       // -- log to array for now
//       time_history[iteration] = loop_start_time;
//       altitude_history[iteration] = state.h;
//       velocity_history[iteration] = state.v;
//       control_input_history[iteration] = u;
//       optimise_time_history[iteration] =
//           (float)(end_time - start_time) / 1000.0f;
//       loop_time_history[iteration] =
//           (float)(loop_end_time - loop_start_time) / 1000.0f;
//       predicted_apogee_history[iteration] = h_pred;

//       iteration++;
//     }
//   }
//   printf("Predicted apogee inital: %.3f\n", h_pred_initial);
//   for (int i = 0; i < iteration; i++) {
//     printf(
//         "t = %.3f, h=%.2f v=%.2f u=%.2f h_pred = %.2f optimise_time(ms) =
//         %.3f " "loop time(ms) = %.3f\n", time_history[i] / 1000000.f,
//         altitude_history[i], velocity_history[i], control_input_history[i],
//         predicted_apogee_history[i], optimise_time_history[i],
//         loop_time_history[i]);
//   }
// }

// // need launch interrupt

// // apogee interrupt

// // coast phase interrupt