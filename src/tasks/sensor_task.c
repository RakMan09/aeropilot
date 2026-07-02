#include <stdio.h>

#include "app/app.h"
#include "sim/rocket_sim.h"

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

    for (;;)
    {
        rocket_sim_advance(&sim, SIM_DT);
        sensor_sample_t sample = rocket_sim_read(&sim);

        (void)xQueueSend(g_sensor_queue, &sample, 0);
        app_checkin(WDT_SENSOR);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SENSOR_PERIOD_MS));
    }
}
