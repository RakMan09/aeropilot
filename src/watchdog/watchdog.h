/* Pure watchdog monitor logic.
 *
 * Each monitored task publishes a monotonically increasing "check-in" counter.
 * Every watchdog period the monitor samples the counters; a task that fails to
 * advance its counter for max_missed consecutive periods is declared stalled.
 * Kept free of RTOS dependencies so the trip logic is unit-testable. */
#ifndef AEROPILOT_WATCHDOG_H
#define AEROPILOT_WATCHDOG_H

#include <stdint.h>

#define WD_MAX_TASKS 8

typedef struct
{
    int      n;                    /* Number of monitored tasks. */
    int      max_missed;           /* Consecutive misses that trip a fault. */
    uint32_t prev[WD_MAX_TASKS];   /* Last observed check-in counters. */
    int      missed[WD_MAX_TASKS]; /* Consecutive missed periods per task. */
    int      primed;               /* prev[] has been seeded. */
} watchdog_t;

void watchdog_init(watchdog_t *wd, int n, int max_missed);

/* Sample the current check-in counters. Returns the index of the first task
 * declared stalled this period, or -1 if all tasks are healthy. */
int watchdog_check(watchdog_t *wd, const uint32_t *current);

#endif /* AEROPILOT_WATCHDOG_H */
