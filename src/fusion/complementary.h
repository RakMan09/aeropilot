/* Complementary filters for AeroPilot.
 *
 * A vertical altitude/velocity complementary filter blends accelerometer
 * integration (good short-term) with barometric altitude (good long-term).
 * An attitude complementary filter blends gyro integration with the
 * accelerometer tilt vector. Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_COMPLEMENTARY_H
#define AEROPILOT_COMPLEMENTARY_H

typedef struct
{
    float alpha;    /* Blend weight toward the inertial prediction (0..1). */
    float altitude; /* Fused altitude (m). */
    float velocity; /* Fused vertical velocity (m/s). */
    float prev_alt; /* Previous baro altitude for velocity derivation. */
    int initialized;
} comp_vertical_t;

void comp_vertical_init(comp_vertical_t *c, float alpha);
void comp_vertical_step(comp_vertical_t *c, float a_vert, float z_alt,
                        float dt);

typedef struct
{
    float alpha; /* Blend weight toward the gyro prediction (0..1). */
    float pitch; /* Estimated pitch (rad). */
    float roll;  /* Estimated roll (rad). */
} comp_attitude_t;

void comp_attitude_init(comp_attitude_t *a, float alpha);
void comp_attitude_step(comp_attitude_t *a, float gx, float gy,
                        float ax, float ay, float az, float dt);

#endif /* AEROPILOT_COMPLEMENTARY_H */
