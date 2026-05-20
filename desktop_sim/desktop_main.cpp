#include <fstream>
#include <iomanip>
#include <iostream>

#include "Dynamics.h"
#include "ekf.h"
#include "hitl_data.h"

int main() {
  std::ofstream out("out_sim.csv");
  out << "timestamp_ms,accelY,imu_velocity,pressureHPa,altitudeM,ekf_height,"
         "ekf_velocity,servo_cmd,pred_apogee\n";

  EKF ekf;
  float dt = 0.01f;
  ekf_init(&ekf, dt, 0.0f);

  float velocity = 0.0f;
  float u_prev = 0.0f;

  std::cout << "[SIM] Starting Desktop Simulation..." << std::endl;

  for (int i = 0; i < HITL_DATA_SIZE; i++) {
    const auto& row = hitl_data[i];

    float accel_vertical = row.accelY - 9.81f;
    float gyro_pitch = row.gyroX;
    float gyro_yaw = row.gyroZ;

    velocity += accel_vertical * dt;
    ekf_predict(&ekf, accel_vertical, gyro_pitch, gyro_yaw);

    if (i % 4 == 0) {  // Every 40ms, simulate 25Hz Barometer limit
      ekf_update(&ekf, row.altitudeM);
    }

    float u = 0.0f;
    float h_pred = 0.0f;

    if (ekf.x[0] > 100.0f) {
      u = OptimiseControlInputBinarySearchConstraint(ekf.x[0], velocity, u_prev,
                                                     0);
      u_prev = u;
      h_pred = PredictApogee(ekf.x[0], velocity, u);
    }

    out << std::fixed << std::setprecision(6) << row.timestamp_ms << ","
        << row.accelY << "," << velocity << "," << row.pressureHPa << ","
        << row.altitudeM << "," << ekf.x[0] << "," << ekf.x[1] << "," << u
        << "," << h_pred << "\n";
  }

  out.close();
  std::cout << "[SIM] Processed " << HITL_DATA_SIZE << " frames instantly!"
            << std::endl;
  std::cout << "[SIM] Simulation complete. Data written to out_sim.csv"
            << std::endl;
  return 0;
}
