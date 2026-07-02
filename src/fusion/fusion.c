#include "fusion/fusion.h"

#include <math.h>

/* Tuning constants. The Kalman Q/R were tuned against the reference
 * trajectory produced by rocket_sim; the complementary alpha favours the
 * inertial channel while letting the barometer bound long-term drift. */
#define FUSION_COMP_ALPHA      0.98f
#define FUSION_ATT_ALPHA       0.98f
#define FUSION_KALMAN_Q_ACCEL  0.5f
#define FUSION_KALMAN_R_BARO   4.0f

void fusion_init(fusion_t *f, fusion_mode_t mode, float ground_pressure)
{
    f->mode = mode;
    f->ground_pressure = ground_pressure;
    f->initialized = 0;
    f->last_t = 0.0f;

    comp_vertical_init(&f->comp, FUSION_COMP_ALPHA);
    comp_attitude_init(&f->attitude, FUSION_ATT_ALPHA);
    kalman1d_init(&f->kalman, FUSION_KALMAN_Q_ACCEL, FUSION_KALMAN_R_BARO);
}

state_estimate_t fusion_update(fusion_t *f, const sensor_sample_t *sample)
{
    state_estimate_t est;
    float dt;

    if (!f->initialized)
    {
        f->last_t = sample->t;
        f->initialized = 1;
        dt = 0.0f;
    }
    else
    {
        dt = sample->t - f->last_t;
        f->last_t = sample->t;
    }

    if (dt <= 0.0f)
    {
        dt = 1e-3f; /* Guard against a zero/negative step. */
    }

    float baro_alt = baro_pressure_to_altitude(sample->pressure,
                                               f->ground_pressure);

    /* Vertical acceleration in the world frame: the accelerometer measures
     * specific force, so subtract gravity to recover kinematic accel. Assumes
     * near-vertical flight (small tilt) which holds for a stable rocket. */
    float a_vert = sample->az - AEROPILOT_GRAVITY_MS2;

    comp_attitude_step(&f->attitude, sample->gx, sample->gy,
                       sample->ax, sample->ay, sample->az, dt);

    if (f->mode == FUSION_KALMAN)
    {
        kalman1d_step(&f->kalman, a_vert, baro_alt, dt);
        est.altitude = kalman1d_altitude(&f->kalman);
        est.velocity = kalman1d_velocity(&f->kalman);
    }
    else
    {
        comp_vertical_step(&f->comp, a_vert, baro_alt, dt);
        est.altitude = f->comp.altitude;
        est.velocity = f->comp.velocity;
    }

    est.t = sample->t;
    est.pitch = f->attitude.pitch;
    est.roll = f->attitude.roll;
    return est;
}
