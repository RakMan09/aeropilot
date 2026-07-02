#include <stdio.h>

#include "app/app.h"
#include "state/flight_sm.h"

/* Drives the flight state machine from fused estimates, logs state
 * transitions, updates the shared vehicle state for telemetry, and honours a
 * watchdog-requested transition to SAFE. */
void state_machine_task(void *arg)
{
    (void)arg;

    flight_sm_t sm;
    flight_sm_config_t cfg;
    flight_sm_default_config(&cfg);
    flight_sm_init(&sm, &cfg);
    flight_sm_arm(&sm);
    printf("[sm] armed; awaiting launch\n");

    flight_state_t prev = sm.state;

    for (;;)
    {
        estimate_msg_t msg;
        if (xQueueReceive(g_estimate_queue, &msg, portMAX_DELAY) == pdTRUE)
        {
            if (g_fault_requested)
            {
                flight_sm_fault(&sm);
            }

            flight_state_t state = flight_sm_update(&sm, &msg.est, msg.a_vert);

            if (state != prev)
            {
                printf("[sm] %s -> %s @ t=%.2fs alt=%.1fm vel=%.1fm/s\n",
                       flight_state_name(prev), flight_state_name(state),
                       (double)msg.est.t, (double)msg.est.altitude,
                       (double)msg.est.velocity);
                if (state == FLIGHT_APOGEE && flight_sm_deploy_fired(&sm))
                {
                    printf("[sm] DEPLOY parachute @ t=%.2fs alt=%.1fm\n",
                           (double)sm.deploy_time, (double)sm.max_altitude);
                }
                prev = state;
            }

            if (xSemaphoreTake(g_shared.lock, portMAX_DELAY) == pdTRUE)
            {
                g_shared.est = msg.est;
                g_shared.state = state;
                g_shared.deployed = flight_sm_deploy_fired(&sm);
                g_shared.safe = (state == FLIGHT_SAFE);
                g_shared.deploy_time = sm.deploy_time;
                g_shared.apogee_alt = sm.max_altitude;
                (void)xSemaphoreGive(g_shared.lock);
            }

            app_checkin(WDT_STATE);

            if (state == FLIGHT_LANDED)
            {
                printf("[sm] LANDED. apogee=%.1fm deploy@t=%.2fs\n",
                       (double)sm.max_altitude, (double)sm.deploy_time);
                vTaskDelay(pdMS_TO_TICKS(200)); /* flush final telemetry */
                printf("FLIGHT COMPLETE\n");
                qemu_exit(0);
            }
        }
    }
}
