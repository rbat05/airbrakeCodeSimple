// #include "ekf.h"

// EKF ekf;   // global — must persist between loop() calls

// void setup() {
//     // --- initialise sensors ---
//     init_mpu6050();
//     init_bme280();

//     // --- get first baro reading as height reference ---
//     float h0 = read_baro_height();

//     // --- initialise filter ONCE ---
//     float dt = 0.01f;   // 100Hz IMU rate (Can control to be loop rate)
//     ekf_init(&ekf, dt, h0);
// }

// void loop() {
//     // --- always: read IMU and run predict ---
//     float a_body  = read_accel_z();    // m/s^2
//     float w_pitch = read_gyro_y();     // rad/s
//     float w_yaw   = read_gyro_z();     // rad/s
//     ekf_predict(&ekf, a_body, w_pitch, w_yaw);

//     // --- only when baro has new data: run update ---
//     if (baro_data_ready()) {
//         float z_baro = read_baro_height();   // metres
//         ekf_update(&ekf, z_baro);
//     }

//     // --- read outputs any time after first predict ---
//     float height   = ekf_get_height(&ekf);
//     float velocity = ekf_get_velocity(&ekf);
// }

// Option 1: check baro status register to see if new data is ready before reading
// bool baro_data_ready() {
//     // BME280 status register 0xF3
//     // bit 3 = measuring flag, bit 0 = NVM copying flag
//     // when both are 0, a fresh result is available
//     Wire.beginTransmission(0x76);   // BME280 I2C address
//     Wire.write(0xF3);               // status register address
//     Wire.endTransmission(false);
//     Wire.requestFrom(0x76, 1);
//     uint8_t status = Wire.read();
//     return (status & 0x09) == 0;    // bits 3 and 0 both clear = ready
// }

// Option 2: just read at a fixed rate (e.g. 25Hz) and assume data is ready by then
// unsigned long last_baro_us = 0;

// void loop() {
//     unsigned long now = micros();

//     // IMU predict — every loop
//     float dt = (now - last_imu_us) * 1e-6f;
//     last_imu_us = now;
//     ekf_predict(&ekf, a_body, w_pitch, w_yaw, dt);

//     // Baro update — every 40ms (25Hz)
//     if (now - last_baro_us >= 40000UL) {
//         last_baro_us = now;
//         float z_baro = bme.readAltitude(baro_ref_pressure);
//         ekf_update(&ekf, z_baro);
//     }
// }