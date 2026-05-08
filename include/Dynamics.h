#ifndef DYNAMICS_H
#define DYNAMICS_H

// ---- helpers ----
float ABS(float a);
float MIN(float a, float b);
float MAX(float a, float b);
float f1(float s1, float s2);
float f2(float s1, float s2, float Ft, float Cd, float m);
void rk4(float *s1, float *s2, float Ft, float Cd, float m, float dt);
float GetCd(float u);
float PredictApogee(float h, float v, float u);
float OptimiseControlInput(float h, float v, float u_prev);
int LaunchDetected();
int CoastDetected();
int ApogeeDetected();
float OptimiseControlInputBinarySearch(float h, float v, float u_prev);
float OptimiseControlInputBinarySearchConstraint(float h, float v, float u_prev,int iteration);

#endif