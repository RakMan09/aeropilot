/* Deterministic 1-DOF model-rocket flight simulation.
 *
 * Integrates thrust, aerodynamic drag and gravity to produce a vertical
 * trajectory, then synthesises IMU + barometer samples (optionally with
 * pseudo-random noise) for the sensor task. Pure C11, no RTOS dependency.
 */
#ifndef AEROPILOT_ROCKET_SIM_H
#define AEROPILOT_ROCKET_SIM_H

#include "aeropilot.h"

typedef struct
{
    float dry_mass;        /* Airframe mass without propellant (kg). */
    float propellant_mass; /* Propellant burned over the motor burn (kg). */
    float thrust;          /* Average motor thrust during burn (N). */
    float burn_time;       /* Motor burn duration (s). */
    float drag_k;          /* Lumped drag: F_drag = drag_k * v * |v| (N). */
    float ground_pressure; /* Static pressure on the pad (Pa). */
    float accel_noise;     /* Accelerometer noise std-dev (m/s^2). */
    float baro_noise;      /* Barometer noise std-dev (Pa). */
    uint32_t seed;         /* PRNG seed for reproducible noise. */
} rocket_sim_params_t;

typedef struct
{
    rocket_sim_params_t params;
    float t;         /* Current sim time (s). */
    float altitude;  /* True altitude above launch (m). */
    float velocity;  /* True vertical velocity (m/s). */
    float accel;     /* True kinematic vertical acceleration (m/s^2). */
    int landed;      /* Non-zero once the rocket has returned to ground. */
    uint32_t rng;    /* PRNG state. */
} rocket_sim_t;

/* Populate params with a sensible default sport-rocket configuration. */
void rocket_sim_default_params(rocket_sim_params_t *params);

void rocket_sim_init(rocket_sim_t *sim, const rocket_sim_params_t *params);

/* Advance the true physics by dt seconds (fixed-step RK-free integration). */
void rocket_sim_advance(rocket_sim_t *sim, float dt);

/* Produce a (possibly noisy) sensor sample for the current sim state. */
sensor_sample_t rocket_sim_read(rocket_sim_t *sim);

/* Return non-zero once the vehicle has landed (descent complete). */
int rocket_sim_landed(const rocket_sim_t *sim);

/* Compute the true apogee altitude/time by running a private high-resolution
 * copy of the model. Useful for verifying apogee-detection accuracy. */
void rocket_sim_true_apogee(const rocket_sim_params_t *params,
                            float *apogee_alt, float *apogee_time);

#endif /* AEROPILOT_ROCKET_SIM_H */
