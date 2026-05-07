// ============================================================
// ekf.c — Extended Kalman Filter implementation
//
// This file contains ALL function definitions.
// It includes ekf.h to get the struct definition and constants.
//
// The matrix utility functions (mat_zero, mat_copy etc.) are
// declared static — private to this file, not visible externally.
// ============================================================

#include "ekf.h"      // EKF struct, constants, and public function declarations    
#include <math.h>     // cosf(), sinf(), sqrtf()
#include <string.h>   // memset(), memcpy()


// ============================================================
// PRIVATE MATRIX UTILITIES
//
// static means these functions are only visible inside ekf.c
// They are internal helpers — not part of the public interface.
// ============================================================

// Fill every element of M with zero
static void mat_zero(float M[EKF_N][EKF_N]) {
    memset(M, 0, sizeof(float) * EKF_N * EKF_N);
}

// Copy src into dst
static void mat_copy(float dst[EKF_N][EKF_N], float src[EKF_N][EKF_N]) {
    memcpy(dst, src, sizeof(float) * EKF_N * EKF_N);
}

// C = A * B  (standard matrix multiply with tmp buffer to avoid aliasing)
static void mat_mul(float C[EKF_N][EKF_N],
                    float A[EKF_N][EKF_N],
                    float B[EKF_N][EKF_N]) {
    float tmp[EKF_N][EKF_N];
    mat_zero(tmp);
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            for (int k = 0; k < EKF_N; k++)
                tmp[i][j] += A[i][k] * B[k][j];
    mat_copy(C, tmp);
}

// T = A^T  (transpose: swap rows and columns)
static void mat_transpose(float T[EKF_N][EKF_N], float A[EKF_N][EKF_N]) {
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            T[i][j] = A[j][i];
}

// C = A + B  (element-wise addition)
static void mat_add(float C[EKF_N][EKF_N],
                    float A[EKF_N][EKF_N],
                    float B[EKF_N][EKF_N]) {
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            C[i][j] = A[i][j] + B[i][j];
}

// ============================================================
// PRIVATE: COMPUTE NOISE MATRICES FROM SENSOR SPECS
//
// Derives Q and R from physical sensor parameters.
// Called once internally by ekf_init().
// ============================================================

static void ekf_compute_noise_matrices(EKF *ekf) {

    float dt = ekf->dt;

    // --- ACCELEROMETER ---
    // sigma_accel: noise per sample in m/s^2
    //   noise_density (g/sqrt(Hz)) * sqrt(sample_rate) * G -> m/s^2
    float sigma_accel = ACCEL_NOISE_DENSITY * sqrtf(ACCEL_SAMPLE_RATE) * G;

    // Velocity uncertainty: integrating accel noise once scales by dt
    float q_hdot = (sigma_accel * dt) * (sigma_accel * dt);

    // Height uncertainty: integrating accel noise twice scales by 0.5*dt^2
    float q_h = (sigma_accel * 0.5f * dt * dt) * (sigma_accel * 0.5f * dt * dt);

    // Accel bias random walk: convert g -> m/s^2 and square for variance
    float sigma_bacc = ACCEL_BIAS_INSTABILITY * G;
    float q_bacc     = sigma_bacc * sigma_bacc;

    // --- GYROSCOPE ---
    // sigma_gyro: noise per sample in rad/s
    float sigma_gyro = GYRO_NOISE_DENSITY * (PI / 180.0f) * sqrtf(GYRO_SAMPLE_RATE);

    // Attitude uncertainty: integrating gyro noise once scales     #define PI() 3.14159265358979323846f    #define PI() 3.14159265358979323846fby dt
    // Same value for pitch and yaw — same physical sensor
    float q_theta = (sigma_gyro * dt) * (sigma_gyro * dt);

    // Gyro bias random walk: convert deg/s -> rad/s and square
    float sigma_bgyro = GYRO_BIAS_INSTABILITY * (PI / 180.0f);
    float q_bgyro     = sigma_bgyro * sigma_bgyro;

    // --- BUILD Q (diagonal) ---
    // Each diagonal entry = variance of noise added to that state per step.
    // Off-diagonal = zero (noise sources are independent).
    mat_zero(ekf->Q);
    ekf->Q[0][0] = q_h;       // height
    ekf->Q[1][1] = q_hdot;    // vertical velocity
    ekf->Q[2][2] = q_theta;   // pitch angle
    ekf->Q[3][3] = q_theta;   // yaw angle — same noise spec as pitch
    ekf->Q[4][4] = q_bgyro;   // pitch gyro bias drift
    ekf->Q[5][5] = q_bgyro;   // yaw gyro bias drift
    ekf->Q[6][6] = q_bacc;    // accel bias drift (large for MPU-6050)

    // --- R (scalar) ---
    // Baro height variance: R = sigma^2 = 1.7^2 = 2.89 m^2
    ekf->R = BARO_RMS_NOISE * BARO_RMS_NOISE;
}

// ============================================================
// PUBLIC: INITIALISE EKF
// ============================================================

void ekf_init(EKF *ekf, float dt_seconds, float h0) {

    ekf->dt = dt_seconds;

    // Initial state — stationary and vertical on pad
    // Biases start at zero and converge over the first few seconds
    ekf->x[0] = h0;    // height from first baro reading
    ekf->x[1] = 0.0f;  // velocity: zero
    ekf->x[2] = 0.0f;  // pitch: zero (vertical)
    ekf->x[3] = 0.0f;  // yaw: zero (vertical)
    ekf->x[4] = 0.0f;  // pitch gyro bias: unknown, start at zero
    ekf->x[5] = 0.0f;  // yaw gyro bias: unknown, start at zero
    ekf->x[6] = 0.0f;  // accel bias: unknown, start at zero

    // Initial covariance P — diagonal, reflects startup uncertainty
    // sqrt(P[i][i]) ~ ±1 sigma on state i
    mat_zero(ekf->P);
    ekf->P[0][0] = 25.0f;   // height: ±5m  (baro absolute accuracy is poor)
    ekf->P[1][1] = 0.1f;    // velocity: ±0.3m/s
    ekf->P[2][2] = 0.01f;   // pitch: ±0.1rad (~6deg)
    ekf->P[3][3] = 0.01f;   // yaw: ±0.1rad (~6deg)
    ekf->P[4][4] = 0.01f;   // pitch gyro bias: wide prior
    ekf->P[5][5] = 0.01f;   // yaw gyro bias: wide prior
    ekf->P[6][6] = 1.0f;    // accel bias: wide prior (MPU-6050 bias is unreliable)

    // Compute Q and R from sensor parameters
    ekf_compute_noise_matrices(ekf);
}

// ============================================================
// PUBLIC: PREDICT STEP
// Call every IMU sample (fast loop, e.g. 100Hz)
// ============================================================

void ekf_predict(EKF *ekf, float a_body, float w_pitch, float w_yaw) {

    float dt = ekf->dt;

    // Unpack current state estimate
    float h        = ekf->x[0];
    float h_dot    = ekf->x[1];
    float theta_p  = ekf->x[2];
    float theta_y  = ekf->x[3];
    float b_gyro_p = ekf->x[4];
    float b_gyro_y = ekf->x[5];
    float b_acc    = ekf->x[6];

    // ----------------------------------------------------------
    // STATE PREDICTION: x̂⁻ = f(x, u)
    // Full nonlinear physics — Jacobian NOT used here
    // ----------------------------------------------------------

    // Remove estimated bias from raw IMU readings
    float a_true  = a_body  - b_acc;
    float wp_true = w_pitch - b_gyro_p;
    float wy_true = w_yaw   - b_gyro_y;

    // Integrate gyro to update both tilt angles
    float theta_p_new = theta_p + wp_true * dt;
    float theta_y_new = theta_y + wy_true * dt;

    // Precompute trig — used in both state prediction and Jacobian
    float cos_tp = cosf(theta_p_new);
    float sin_tp = sinf(theta_p_new);
    float cos_ty = cosf(theta_y_new);
    float sin_ty = sinf(theta_y_new);

    // Project body-frame accel into world vertical frame
    // a_vertical = a_true * cos(theta_p) * cos(theta_y)
    // When both angles zero: full accel is vertical (correct)
    // When either angle 90deg: no vertical component (correct)
    float a_net = a_true * cos_tp * cos_ty - G;

    // Integrate to get velocity and height
    float h_dot_new = h_dot + a_net * dt;
    float h_new     = h + h_dot * dt + 0.5f * a_net * dt * dt;

    // Biases held constant — corrected later in update step
    // Q adds uncertainty to these states each step to keep filter
    // receptive to correcting them during the update
    float b_gyro_p_new = b_gyro_p;
    float b_gyro_y_new = b_gyro_y;
    float b_acc_new    = b_acc;

    // Write predicted state x̂⁻
    ekf->x[0] = h_new;
    ekf->x[1] = h_dot_new;
    ekf->x[2] = theta_p_new;
    ekf->x[3] = theta_y_new;
    ekf->x[4] = b_gyro_p_new;
    ekf->x[5] = b_gyro_y_new;
    ekf->x[6] = b_acc_new;

    // ----------------------------------------------------------
    // JACOBIAN F = df/dx evaluated at current state (7x7)
    // Used ONLY to propagate covariance — not the state itself.
    // F[i][j] = partial derivative of state output i w.r.t. state j
    //
    // Key nonlinear entries come from the cos(tp)*cos(ty) projection:
    //   dh/dtheta_p    = -0.5 * a_true * sin(tp) * cos(ty) * dt^2
    //   dh/dtheta_y    = -0.5 * a_true * cos(tp) * sin(ty) * dt^2
    //   dhdot/dtheta_p = -a_true * sin(tp) * cos(ty) * dt
    //   dhdot/dtheta_y = -a_true * cos(tp) * sin(ty) * dt
    //   dh/db_acc      = -0.5 * cos(tp) * cos(ty) * dt^2
    //   dhdot/db_acc   = -cos(tp) * cos(ty) * dt
    // ----------------------------------------------------------

    float F[EKF_N][EKF_N];
    mat_zero(F);

    // Precompute repeated product terms
    float stp_cty = sin_tp * cos_ty;   // d/dtheta_p of (cos_tp * cos_ty)
    float ctp_sty = cos_tp * sin_ty;   // d/dtheta_y of (cos_tp * cos_ty)
    float ctp_cty = cos_tp * cos_ty;   // the projection — used for bias derivative

    // Row 0: height
    F[0][0] = 1.0f;
    F[0][1] = dt;
    F[0][2] = -0.5f * a_true * stp_cty * dt * dt;   // dh/dtheta_p
    F[0][3] = -0.5f * a_true * ctp_sty * dt * dt;   // dh/dtheta_y
    F[0][4] = 0.0f;
    F[0][5] = 0.0f;
    F[0][6] = -0.5f * ctp_cty * dt * dt;            // dh/db_acc

    // Row 1: velocity
    F[1][0] = 0.0f;
    F[1][1] = 1.0f;
    F[1][2] = -a_true * stp_cty * dt;               // dhdot/dtheta_p
    F[1][3] = -a_true * ctp_sty * dt;               // dhdot/dtheta_y
    F[1][4] = 0.0f;
    F[1][5] = 0.0f;
    F[1][6] = -ctp_cty * dt;                        // dhdot/db_acc

    // Row 2: pitch angle
    F[2][2] = 1.0f;   // dtp/dtheta_p
    F[2][4] = -dt;    // dtp/db_gyro_p

    // Row 3: yaw angle
    F[3][3] = 1.0f;   // dty/dtheta_y
    F[3][5] = -dt;    // dty/db_gyro_y

    // Rows 4, 5, 6: biases are identity (b_new = b_old)
    F[4][4] = 1.0f;
    F[5][5] = 1.0f;
    F[6][6] = 1.0f;

    // ----------------------------------------------------------
    // COVARIANCE PREDICT: P⁻ = F * P * F^T + Q
    // F*P*F^T propagates uncertainty through the dynamics.
    // Q adds fresh uncertainty from IMU noise this step.
    // ----------------------------------------------------------

    float Ft[EKF_N][EKF_N];
    float FP[EKF_N][EKF_N];
    float FPFt[EKF_N][EKF_N];

    mat_transpose(Ft, F);
    mat_mul(FP, F, ekf->P);
    mat_mul(FPFt, FP, Ft);
    mat_add(ekf->P, FPFt, ekf->Q);
}

// ============================================================
// PUBLIC: UPDATE STEP
// Call when a new baro reading is available (slow loop, e.g. 25Hz)
// ============================================================

void ekf_update(EKF *ekf, float z_baro) {

    // Measurement model: z = H * x
    // H = [1, 0, 0, 0, 0, 0, 0] — baro only observes height (x[0])
    // Linear model — no Jacobian needed here

    // Innovation: difference between baro reading and predicted height
    float y = z_baro - ekf->x[0];

    // Innovation covariance S = H*P*H^T + R
    // H=[1,0,...,0] so H*P*H^T = P[0][0]
    float S = ekf->P[0][0] + ekf->R;

    // Kalman gain K = P*H^T / S
    // H^T=[1,0,...,0]^T so P*H^T = first column of P
    // K[i] reflects how correlated state i is with height
    float K[EKF_N];
    for (int i = 0; i < EKF_N; i++)
        K[i] = ekf->P[i][0] / S;

    // State update: x = x̂⁻ + K * y
    // Distributes baro innovation across all 7 states
    // Biases x[4], x[5], x[6] are nudged here via K[4..6]*y
    for (int i = 0; i < EKF_N; i++)
        ekf->x[i] += K[i] * y;

    // Covariance update: P = (I - K*H) * P⁻
    // K*H is 7x7: column 0 = K, all other columns = 0
    float KH[EKF_N][EKF_N];
    mat_zero(KH);
    for (int i = 0; i < EKF_N; i++)
        KH[i][0] = K[i];

    // (I - K*H)
    float IKH[EKF_N][EKF_N];
    mat_zero(IKH);
    for (int i = 0; i < EKF_N; i++)
        IKH[i][i] = 1.0f;
    for (int i = 0; i < EKF_N; i++)
        for (int j = 0; j < EKF_N; j++)
            IKH[i][j] -= KH[i][j];

    // P = (I - K*H) * P⁻
    float P_new[EKF_N][EKF_N];
    mat_mul(P_new, IKH, ekf->P);
    mat_copy(ekf->P, P_new);
}

// ============================================================
// PUBLIC: ACCESSORS
// ============================================================

float ekf_get_height(EKF *ekf)    { return ekf->x[0]; }
float ekf_get_velocity(EKF *ekf)  { return ekf->x[1]; }
float ekf_get_pitch(EKF *ekf)     { return ekf->x[2]; }
float ekf_get_yaw(EKF *ekf)       { return ekf->x[3]; }
float ekf_get_b_gyro_p(EKF *ekf)  { return ekf->x[4]; }
float ekf_get_b_gyro_y(EKF *ekf)  { return ekf->x[5]; }
float ekf_get_b_acc(EKF *ekf)     { return ekf->x[6]; }