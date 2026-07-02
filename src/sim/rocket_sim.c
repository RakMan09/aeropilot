#include "sim/rocket_sim.h"

#include <math.h>
#include <stddef.h>

/* Simple, deterministic linear-congruential PRNG so that noisy runs are
 * exactly reproducible across host and target builds. */
static uint32_t lcg_next(uint32_t *state)
{
    *state = (*state * 1664525U) + 1013904223U;
    return *state;
}

/* Approximate a zero-mean unit-variance Gaussian via the central-limit sum of
 * uniform samples (sufficient fidelity for sensor noise modelling). */
static float gauss(uint32_t *state)
{
    float sum = 0.0f;
    for (int i = 0; i < 12; ++i)
    {
        sum += (float)(lcg_next(state) & 0xFFFFFFU) / (float)0x1000000U;
    }
    return sum - 6.0f;
}

void rocket_sim_default_params(rocket_sim_params_t *params)
{
    params->dry_mass = 0.40f;
    params->propellant_mass = 0.05f;
    params->thrust = 30.0f;      /* ~ an Estes F-class motor. */
    params->burn_time = 1.0f;
    params->drag_k = 0.003f;     /* Lumped drag -> ~150 m apogee, ~15 s flight. */
    params->ground_pressure = AEROPILOT_SEA_LEVEL_PRESSURE_PA;
    params->accel_noise = 0.15f;
    params->baro_noise = 25.0f;
    params->seed = 0x1234ABCDU;
}

void rocket_sim_init(rocket_sim_t *sim, const rocket_sim_params_t *params)
{
    sim->params = *params;
    sim->t = 0.0f;
    sim->altitude = 0.0f;
    sim->velocity = 0.0f;
    sim->accel = 0.0f;
    sim->landed = 0;
    sim->rng = params->seed;
}

/* Total instantaneous mass (propellant burns off linearly during the burn). */
static float current_mass(const rocket_sim_t *sim, float t)
{
    const rocket_sim_params_t *p = &sim->params;
    if (t >= p->burn_time)
    {
        return p->dry_mass;
    }
    float frac = 1.0f - (t / p->burn_time);
    return p->dry_mass + (p->propellant_mass * frac);
}

/* Kinematic vertical acceleration (excludes the reaction that keeps the
 * rocket on the pad before liftoff). */
static float kinematic_accel(const rocket_sim_t *sim, float t,
                             float altitude, float velocity)
{
    const rocket_sim_params_t *p = &sim->params;
    float mass = current_mass(sim, t);
    float thrust = (t < p->burn_time) ? p->thrust : 0.0f;
    float drag = p->drag_k * velocity * fabsf(velocity); /* opposes motion */
    float force = thrust - drag - (mass * AEROPILOT_GRAVITY_MS2);

    /* Hold on the pad until thrust overcomes weight. */
    if (altitude <= 0.0f && force <= 0.0f)
    {
        return 0.0f;
    }
    return force / mass;
}

void rocket_sim_advance(rocket_sim_t *sim, float dt)
{
    if (sim->landed)
    {
        /* Keep the mission clock running while resting on the ground so
         * downstream timestamps (and thus the fusion dt) stay well-defined. */
        sim->t += dt;
        return;
    }

    /* Semi-implicit Euler integration at the caller's step. */
    float a = kinematic_accel(sim, sim->t, sim->altitude, sim->velocity);
    sim->accel = a;
    sim->velocity += a * dt;
    sim->altitude += sim->velocity * dt;
    sim->t += dt;

    if (sim->altitude <= 0.0f && sim->t > sim->params.burn_time)
    {
        sim->altitude = 0.0f;
        sim->velocity = 0.0f;
        sim->accel = 0.0f;
        sim->landed = 1;
    }
}

sensor_sample_t rocket_sim_read(rocket_sim_t *sim)
{
    sensor_sample_t s;
    const rocket_sim_params_t *p = &sim->params;

    /* Accelerometer measures specific force = kinematic accel + gravity. */
    float specific_force = sim->accel + AEROPILOT_GRAVITY_MS2;

    /* Barometric pressure from the true altitude (inverse of the barometric
     * altitude formula), referenced to the pad pressure. */
    float exponent = 5.255f;
    float ratio = 1.0f - (sim->altitude / 44330.0f);
    if (ratio < 0.0f)
    {
        ratio = 0.0f;
    }
    float pressure = p->ground_pressure * powf(ratio, exponent);

    s.t = sim->t;
    s.ax = gauss(&sim->rng) * p->accel_noise;
    s.ay = gauss(&sim->rng) * p->accel_noise;
    s.az = specific_force + (gauss(&sim->rng) * p->accel_noise);
    s.gx = gauss(&sim->rng) * 0.01f;
    s.gy = gauss(&sim->rng) * 0.01f;
    s.gz = gauss(&sim->rng) * 0.01f;
    s.pressure = pressure + (gauss(&sim->rng) * p->baro_noise);
    return s;
}

int rocket_sim_landed(const rocket_sim_t *sim)
{
    return sim->landed;
}

void rocket_sim_true_apogee(const rocket_sim_params_t *params,
                            float *apogee_alt, float *apogee_time)
{
    rocket_sim_params_t clean = *params;
    clean.accel_noise = 0.0f;
    clean.baro_noise = 0.0f;

    rocket_sim_t sim;
    rocket_sim_init(&sim, &clean);

    float best_alt = 0.0f;
    float best_t = 0.0f;
    const float dt = 0.0005f;

    while (!sim.landed && sim.t < 120.0f)
    {
        rocket_sim_advance(&sim, dt);
        if (sim.altitude > best_alt)
        {
            best_alt = sim.altitude;
            best_t = sim.t;
        }
    }

    if (apogee_alt != NULL)
    {
        *apogee_alt = best_alt;
    }
    if (apogee_time != NULL)
    {
        *apogee_time = best_t;
    }
}
