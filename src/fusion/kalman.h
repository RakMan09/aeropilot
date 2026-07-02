/* 1-D Kalman filter estimating altitude and vertical velocity from a noisy
 * barometric altitude measurement and an accelerometer-derived control input.
 * Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_KALMAN_H
#define AEROPILOT_KALMAN_H

typedef struct
{
    float x[2];     /* State: [altitude (m), vertical velocity (m/s)]. */
    float P[2][2];  /* State covariance. */
    float q_accel;  /* Process noise: variance of the accel input (m/s^2)^2. */
    float r_baro;   /* Measurement noise: baro altitude variance (m^2). */
} kalman1d_t;

/* Initialise the filter. q_accel models trust in the accel-driven prediction;
 * r_baro models trust in the barometer measurement. */
void kalman1d_init(kalman1d_t *kf, float q_accel, float r_baro);

/* Advance one step: predict using vertical acceleration a_vert over dt, then
 * correct with a barometric altitude measurement z_alt. */
void kalman1d_step(kalman1d_t *kf, float a_vert, float z_alt, float dt);

float kalman1d_altitude(const kalman1d_t *kf);
float kalman1d_velocity(const kalman1d_t *kf);

#endif /* AEROPILOT_KALMAN_H */
