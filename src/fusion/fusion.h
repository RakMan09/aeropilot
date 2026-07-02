/* Unified sensor-fusion front end. Selects either the complementary or the
 * Kalman altitude/velocity estimator and always runs the attitude
 * complementary filter, producing a state_estimate_t from raw samples.
 * Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_FUSION_H
#define AEROPILOT_FUSION_H

#include "aeropilot.h"
#include "fusion/complementary.h"
#include "fusion/kalman.h"

typedef enum
{
    FUSION_COMPLEMENTARY = 0,
    FUSION_KALMAN = 1
} fusion_mode_t;

typedef struct
{
    fusion_mode_t mode;
    float ground_pressure; /* Pad pressure for baro->altitude conversion. */
    int initialized;
    float last_t;

    comp_vertical_t comp;
    comp_attitude_t attitude;
    kalman1d_t kalman;
} fusion_t;

/* Initialise the fusion engine. ground_pressure is the pressure measured on
 * the launch pad, used to reference altitude to zero at launch. */
void fusion_init(fusion_t *f, fusion_mode_t mode, float ground_pressure);

/* Feed one raw sensor sample; returns the current fused state estimate. */
state_estimate_t fusion_update(fusion_t *f, const sensor_sample_t *sample);

#endif /* AEROPILOT_FUSION_H */
