// ============================================================
// test_ekf.c — Offline replay harness for the EKF
//
// Reads a logged sensor CSV (timestamp_ms,accelX,accelY,accelZ,
// gyroX,gyroY,gyroZ,pressureHPa,altitudeM,...) and replays it through
// EXACTLY the same predict/update/flight-phase-gating logic as
// main.cpp, but on your PC instead of the ESP32. Writes a new CSV
// with the EKF's height/velocity output so you can plot it in MATLAB
// next to the original filteredHeight/filteredVelocity columns to
// compare before vs. after the fix.
//
// Build:
//   g++ -O2 -o test_ekf -Iinclude src/test_ekf.cpp src/ekf.cpp -lm
//
// Run:
//   ./test_ekf time_filtered.csv ekf_replay_output.csv
// ============================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ekf.h"

#define MAX_LINE 1024
#define MAX_FIELDS 16

// ---- must mirror the constants/logic in main.cpp exactly ----
typedef enum { PHASE_PAD, PHASE_BURN, PHASE_COAST } FlightPhase;

static const float LAUNCH_ACCEL_THRESHOLD = 15.0f;
static const float BURNOUT_ACCEL_THRESHOLD = 2.0f;
static const int CONFIRM_SAMPLES = 15;
static const float BURN_R_SCALE = 500.0f;

// Once burnout is confirmed, ease R back down to nominal over this many
// predict steps instead of switching instantly. This gives P time to be
// pulled back down by a sequence of small, well-behaved updates rather
// than absorbing several seconds' worth of accumulated baro-vs-model
// disagreement (including real dynamic-pressure bias in the baro) in a
// single oversized correction.
static const int RAMP_SAMPLES = 60;  // ~0.6s @ 100Hz — tune against your data
static int ramp_remaining = 0;

static FlightPhase update_flight_phase(EKF* ekf, FlightPhase phase,
                                       int* confirm_count,
                                       float accel_vertical_raw) {
  float a_net_approx = accel_vertical_raw - G;

  switch (phase) {
    case PHASE_PAD:
      if (a_net_approx > LAUNCH_ACCEL_THRESHOLD) {
        phase = PHASE_BURN;
        *confirm_count = 0;
      }
      break;
    case PHASE_BURN:
      if (a_net_approx < BURNOUT_ACCEL_THRESHOLD) {
        (*confirm_count)++;
        if (*confirm_count > CONFIRM_SAMPLES) {
          phase = PHASE_COAST;
          ramp_remaining = RAMP_SAMPLES;  // start easing trust back in
        }
      } else {
        *confirm_count = 0;
      }
      break;
    case PHASE_COAST:
      break;
  }

  float r_scale;
  if (phase == PHASE_BURN) {
    r_scale = BURN_R_SCALE;
  } else if (ramp_remaining > 0) {
    // linear ramp from BURN_R_SCALE down to 1.0 over RAMP_SAMPLES steps
    float frac = (float)ramp_remaining / (float)RAMP_SAMPLES;
    r_scale = 1.0f + (BURN_R_SCALE - 1.0f) * frac;
    ramp_remaining--;
  } else {
    r_scale = 1.0f;
  }

  ekf_set_r_scale(ekf, r_scale);
  return phase;
}

// ---- tiny CSV line splitter (handles trailing \r\n) ----
static int split_line(char* line, char* fields[], int max_fields) {
  int n = 0;
  char* tok = strtok(line, ",\r\n");
  while (tok != NULL && n < max_fields) {
    fields[n++] = tok;
    tok = strtok(NULL, ",\r\n");
  }
  return n;
}

int main(int argc, char** argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s <input_csv> <output_csv>\n", argv[0]);
    return 1;
  }

  FILE* fin = fopen(argv[1], "r");
  if (!fin) {
    fprintf(stderr, "Could not open input file %s\n", argv[1]);
    return 1;
  }
  FILE* fout = fopen(argv[2], "w");
  if (!fout) {
    fprintf(stderr, "Could not open output file %s\n", argv[2]);
    fclose(fin);
    return 1;
  }

  char line[MAX_LINE];

  // --- read + discard header row, but locate column indices by name so
  //     this keeps working if column order ever changes ---
  if (!fgets(line, sizeof(line), fin)) {
    fprintf(stderr, "Empty input file\n");
    return 1;
  }
  char header_copy[MAX_LINE];
  strncpy(header_copy, line, sizeof(header_copy));
  char* fields[MAX_FIELDS];
  int nfields = split_line(header_copy, fields, MAX_FIELDS);

  int idx_ts = -1, idx_accelY = -1, idx_gyroX = -1, idx_gyroZ = -1,
      idx_altitudeM = -1;
  for (int i = 0; i < nfields; i++) {
    if (strcmp(fields[i], "timestamp_ms") == 0) idx_ts = i;
    else if (strcmp(fields[i], "accelY") == 0) idx_accelY = i;
    else if (strcmp(fields[i], "gyroX") == 0) idx_gyroX = i;
    else if (strcmp(fields[i], "gyroZ") == 0) idx_gyroZ = i;
    else if (strcmp(fields[i], "altitudeM") == 0) idx_altitudeM = i;
  }
  if (idx_ts < 0 || idx_accelY < 0 || idx_gyroX < 0 || idx_gyroZ < 0 ||
      idx_altitudeM < 0) {
    fprintf(stderr,
            "Could not find required columns in header (need "
            "timestamp_ms, accelY, gyroX, gyroZ, altitudeM)\n");
    return 1;
  }

  // --- init EKF, matching main.cpp's setup() ---
  EKF ekf;
  float dt_nominal = 0.01f;  // 100Hz nominal, same as main.cpp
  float h0 = 0.0f;           // main.cpp hardcodes h0 = 0
  ekf_init(&ekf, dt_nominal, h0);

  FlightPhase phase = PHASE_PAD;
  int confirm_count = 0;
  int loop_count = 0;
  float raw_velocity = 0.0f;  // INS-only comparison estimate, not fed back

  long prev_ts_ms = -1;

  fprintf(fout,
          "timestamp_ms,dt_s,phase,accelY_raw,baro_altitude_m,"
          "ekf_height_m,ekf_velocity_ms,ekf_pitch_rad,ekf_yaw_rad,"
          "imu_velocity_raw_ms,r_scale\n");

  int row = 0;
  while (fgets(line, sizeof(line), fin)) {
    char line_copy[MAX_LINE];
    strncpy(line_copy, line, sizeof(line_copy));
    int n = split_line(line_copy, fields, MAX_FIELDS);
    if (n <= idx_altitudeM) continue;  // skip malformed/short rows

    long ts_ms = atol(fields[idx_ts]);
    float accelY = atof(fields[idx_accelY]);
    float gyroX = atof(fields[idx_gyroX]);
    float gyroZ = atof(fields[idx_gyroZ]);
    float altitudeM = atof(fields[idx_altitudeM]);

    // dt from logged timestamps; fall back to nominal if the log doesn't
    // have a usable delta (duplicate timestamps show up at the very start
    // of this particular log)
    float dt;
    if (prev_ts_ms < 0 || ts_ms <= prev_ts_ms) {
      dt = dt_nominal;
    } else {
      dt = (ts_ms - prev_ts_ms) / 1000.0f;
    }
    prev_ts_ms = ts_ms;
    ekf.dt = dt;

    loop_count++;

    // raw INS-only estimate for comparison, mirrors main.cpp's `velocity`
    raw_velocity += (accelY - G) * dt;

    phase = update_flight_phase(&ekf, phase, &confirm_count, accelY);

    // gyroX -> pitch, gyroZ -> yaw, matching main.cpp's mapping
    ekf_predict(&ekf, accelY, gyroX, gyroZ);

    if (loop_count == 4) {
      ekf_update(&ekf, altitudeM);
      loop_count = 0;
    }

    const char* phase_str =
        (phase == PHASE_PAD) ? "PAD" : (phase == PHASE_BURN) ? "BURN" : "COAST";
    float r_scale_used = ekf.R / ekf.R_base;  // reflects ramp, not just phase

    fprintf(fout, "%ld,%.6f,%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f\n",
            ts_ms, dt, phase_str, accelY, altitudeM, ekf_get_height(&ekf),
            ekf_get_velocity(&ekf), ekf_get_pitch(&ekf), ekf_get_yaw(&ekf),
            raw_velocity, r_scale_used);

    row++;
  }

  fclose(fin);
  fclose(fout);
  fprintf(stderr, "Replayed %d rows -> %s\n", row, argv[2]);
  return 0;
}