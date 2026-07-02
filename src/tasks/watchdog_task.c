#include <stdio.h>

#include "app/app.h"
#include "watchdog/watchdog.h"

/* Highest-priority task. Every period it verifies that the sensor, fusion and
 * state-machine tasks have advanced their check-in counters. If a task stalls
 * for WATCHDOG_MAX_MISSED consecutive periods it requests the SAFE fault
 * state, forces the shared telemetry state to SAFE (fail to a known-good), and
 * logs the fault. */
void watchdog_task(void *arg)
{
    (void)arg;

    watchdog_t wd;
    watchdog_init(&wd, WDT_COUNT, WATCHDOG_MAX_MISSED);

    static const char *const names[WDT_COUNT] = {
        "sensor", "fusion", "state"
    };

    int tripped = 0;
    int post_trip = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        uint32_t snapshot[WDT_COUNT];
        for (int i = 0; i < WDT_COUNT; ++i)
        {
            snapshot[i] = g_checkin[i];
        }

        int faulted = watchdog_check(&wd, snapshot);
        if (faulted >= 0 && !tripped)
        {
            tripped = 1;
            g_fault_requested = 1;

            if (xSemaphoreTake(g_shared.lock, portMAX_DELAY) == pdTRUE)
            {
                g_shared.safe = 1;
                g_shared.state = FLIGHT_SAFE;
                (void)xSemaphoreGive(g_shared.lock);
            }

            printf("[watchdog] FAULT: task '%s' stalled -> SAFE state\n",
                   names[faulted]);
            printf("SAFE STATE ENTERED\n");
        }

        /* Once in SAFE, hold long enough to emit SAFE telemetry, then end the
         * bounded (CI) session. */
        if (tripped && ++post_trip >= 10)
        {
            printf("FLIGHT COMPLETE\n");
            qemu_exit(0);
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(WATCHDOG_PERIOD_MS));
    }
}
