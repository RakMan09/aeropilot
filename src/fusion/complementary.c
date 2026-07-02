#include "fusion/complementary.h"

#include <math.h>

void comp_vertical_init(comp_vertical_t *c, float alpha)
{
    c->alpha = alpha;
    c->altitude = 0.0f;
    c->velocity = 0.0f;
    c->prev_alt = 0.0f;
    c->initialized = 0;
}

void comp_vertical_step(comp_vertical_t *c, float a_vert, float z_alt,
                        float dt)
{
    if (!c->initialized)
    {
        c->altitude = z_alt;
        c->prev_alt = z_alt;
        c->velocity = 0.0f;
        c->initialized = 1;
        return;
    }

    /* Inertial prediction. */
    float alt_pred = c->altitude + (c->velocity * dt) +
                     (0.5f * a_vert * dt * dt);
    float vel_pred = c->velocity + (a_vert * dt);

    /* Barometer-derived velocity for the low-frequency correction. */
    float baro_vel = (z_alt - c->prev_alt) / dt;

    c->altitude = (c->alpha * alt_pred) + ((1.0f - c->alpha) * z_alt);
    c->velocity = (c->alpha * vel_pred) + ((1.0f - c->alpha) * baro_vel);
    c->prev_alt = z_alt;
}

void comp_attitude_init(comp_attitude_t *a, float alpha)
{
    a->alpha = alpha;
    a->pitch = 0.0f;
    a->roll = 0.0f;
}

void comp_attitude_step(comp_attitude_t *a, float gx, float gy,
                        float ax, float ay, float az, float dt)
{
    /* Gyro-integrated prediction. */
    float pitch_pred = a->pitch + (gy * dt);
    float roll_pred = a->roll + (gx * dt);

    /* Accelerometer tilt (valid when net specific force ~ gravity). */
    float pitch_acc = atan2f(-ax, sqrtf((ay * ay) + (az * az)));
    float roll_acc = atan2f(ay, az);

    a->pitch = (a->alpha * pitch_pred) + ((1.0f - a->alpha) * pitch_acc);
    a->roll = (a->alpha * roll_pred) + ((1.0f - a->alpha) * roll_acc);
}
