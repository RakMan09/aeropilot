#include <stdio.h>

#include "app/app.h"
#include "fusion/fusion.h"

#ifndef AEROPILOT_FUSION_MODE
#define AEROPILOT_FUSION_MODE FUSION_KALMAN
#endif

/* The fusion task consumes raw samples, runs the selected estimator (Kalman by
 * default) plus the attitude complementary filter, and forwards a state
 * estimate to the flight state machine. */
void fusion_task(void *arg)
{
    (void)arg;

    fusion_t fusion;
    fusion_init(&fusion, AEROPILOT_FUSION_MODE, AEROPILOT_SEA_LEVEL_PRESSURE_PA);

    for (;;)
    {
        sensor_sample_t sample;
        if (xQueueReceive(g_sensor_queue, &sample, portMAX_DELAY) == pdTRUE)
        {
#ifdef AEROPILOT_INJECT_HANG
            /* Fault injection: stop making progress once airborne so the
             * watchdog observes a stalled fusion task and trips to SAFE. */
            if (sample.t >= (float)AEROPILOT_INJECT_HANG)
            {
                printf("[fusion] INJECTED HANG at t=%.2f\n", (double)sample.t);
                for (;;)
                {
                }
            }
#endif
            estimate_msg_t msg;
            msg.est = fusion_update(&fusion, &sample);
            msg.a_vert = sample.az - AEROPILOT_GRAVITY_MS2;

            (void)xQueueSend(g_estimate_queue, &msg, 0);
            app_checkin(WDT_FUSION);
        }
    }
}
