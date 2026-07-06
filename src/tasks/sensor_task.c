#include <stdint.h>
#include <stdio.h>

#include "app/app.h"
#include "sim/rocket_sim.h"

/* Measure the realised period of this fixed-priority periodic task in RTOS
 * ticks. With vTaskDelayUntil under preemptive fixed-priority scheduling the
 * period should be exactly the target every cycle; any deviation is jitter.
 * (Sub-tick jitter would need the DWT cycle counter, which the QEMU
 * mps2-an385 model does not implement.) */
static void record_jitter(uint32_t delta_ticks)
{
    uint32_t expected = (uint32_t)pdMS_TO_TICKS(SENSOR_PERIOD_MS);
    uint32_t jitter = (delta_ticks > expected) ? (delta_ticks - expected)
                                               : (expected - delta_ticks);
    if (g_jitter_samples == 0u || delta_ticks < g_period_min_cyc)
    {
        g_period_min_cyc = delta_ticks;
    }
    if (g_jitter_samples == 0u || delta_ticks > g_period_max_cyc)
    {
        g_period_max_cyc = delta_ticks;
    }
    if (jitter > g_jitter_max_cyc)
    {
        g_jitter_max_cyc = jitter;
    }
    g_jitter_samples++;
}

/* The sensor task owns the physics simulation. Each period it advances the
 * simulated flight by SIM_DT, samples the (noisy) IMU + barometer, and pushes
 * the sample onto the sensor queue for the fusion task. */
void sensor_task(void *arg)
{
    (void)arg;

    rocket_sim_t sim;
    rocket_sim_params_t params;
    rocket_sim_default_params(&params);
    rocket_sim_init(&sim, &params);

    TickType_t last_wake = xTaskGetTickCount();
    TickType_t last_tick = last_wake;
    int first = 1;

    for (;;)
    {
        TickType_t now_tick = xTaskGetTickCount();
        if (first)
        {
            first = 0;
        }
        else
        {
            record_jitter((uint32_t)(now_tick - last_tick));
        }
        last_tick = now_tick;

        rocket_sim_advance(&sim, SIM_DT);
        sensor_sample_t sample = rocket_sim_read(&sim);

        (void)xQueueSend(g_sensor_queue, &sample, 0);
        app_checkin(WDT_SENSOR);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
