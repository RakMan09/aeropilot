#include "fusion/kalman.h"

void kalman1d_init(kalman1d_t *kf, float q_accel, float r_baro)
{
    kf->x[0] = 0.0f;
    kf->x[1] = 0.0f;
    kf->P[0][0] = 1.0f;
    kf->P[0][1] = 0.0f;
    kf->P[1][0] = 0.0f;
    kf->P[1][1] = 1.0f;
    kf->q_accel = q_accel;
    kf->r_baro = r_baro;
}

void kalman1d_step(kalman1d_t *kf, float a_vert, float z_alt, float dt)
{
    /* --- Predict ---
     * x = F x + B u, with F = [[1, dt],[0,1]], B = [0.5 dt^2, dt]^T. */
    float dt2 = dt * dt;

    kf->x[0] += (kf->x[1] * dt) + (0.5f * a_vert * dt2);
    kf->x[1] += a_vert * dt;

    /* Process noise Q derived from acceleration uncertainty q_accel:
     * Q = G G^T * q_accel, G = [0.5 dt^2, dt]^T. */
    float q00 = 0.25f * dt2 * dt2 * kf->q_accel;
    float q01 = 0.5f * dt2 * dt * kf->q_accel;
    float q11 = dt2 * kf->q_accel;

    /* P = F P F^T + Q. */
    float p00 = kf->P[0][0];
    float p01 = kf->P[0][1];
    float p10 = kf->P[1][0];
    float p11 = kf->P[1][1];

    float np00 = p00 + (dt * (p10 + p01)) + (dt2 * p11) + q00;
    float np01 = p01 + (dt * p11) + q01;
    float np10 = p10 + (dt * p11) + q01;
    float np11 = p11 + q11;

    kf->P[0][0] = np00;
    kf->P[0][1] = np01;
    kf->P[1][0] = np10;
    kf->P[1][1] = np11;

    /* --- Update with measurement z_alt, H = [1, 0]. --- */
    float y = z_alt - kf->x[0];            /* innovation */
    float s = kf->P[0][0] + kf->r_baro;    /* innovation covariance */
    float k0 = kf->P[0][0] / s;            /* Kalman gain */
    float k1 = kf->P[1][0] / s;

    kf->x[0] += k0 * y;
    kf->x[1] += k1 * y;

    /* P = (I - K H) P. */
    float u00 = kf->P[0][0];
    float u01 = kf->P[0][1];
    kf->P[0][0] -= k0 * u00;
    kf->P[0][1] -= k0 * u01;
    kf->P[1][0] -= k1 * u00;
    kf->P[1][1] -= k1 * u01;
}

float kalman1d_altitude(const kalman1d_t *kf)
{
    return kf->x[0];
}

float kalman1d_velocity(const kalman1d_t *kf)
{
    return kf->x[1];
}
