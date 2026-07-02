/* Shared context for the AeroPilot flight application: inter-task queues, the
 * mutex-protected shared vehicle state, watchdog check-in counters and task
 * priorities. This header is firmware-only (pulls in FreeRTOS). */
#ifndef AEROPILOT_APP_H
#define AEROPILOT_APP_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"

#include "aeropilot.h"

/* Task priorities (higher number = higher priority). The watchdog is the
 * highest-priority application task so it always runs to catch stalls. */
#define PRIO_WATCHDOG   (configMAX_PRIORITIES - 1) /* 7 */
#define PRIO_SENSOR     4
#define PRIO_FUSION     3
#define PRIO_STATE      3
#define PRIO_TELEMETRY  2

/* Task periods. Sensor drives the sim clock at 100 Hz of simulated time. */
#define SENSOR_PERIOD_MS     10
#define SIM_DT               0.01f
#define TELEMETRY_PERIOD_MS  50
#define WATCHDOG_PERIOD_MS   100
#define WATCHDOG_MAX_MISSED  3   /* -> ~300 ms deadline to SAFE. */

/* Watchdog-monitored task indices. */
enum
{
    WDT_SENSOR = 0,
    WDT_FUSION = 1,
    WDT_STATE = 2,
    WDT_COUNT = 3
};

typedef struct
{
    state_estimate_t est;
    float a_vert;
} estimate_msg_t;

typedef struct
{
    SemaphoreHandle_t lock;
    state_estimate_t est;
    flight_state_t state;
    int deployed;
    int safe;
    float deploy_time;
    float apogee_alt;
} shared_state_t;

extern QueueHandle_t g_sensor_queue;
extern QueueHandle_t g_estimate_queue;
extern shared_state_t g_shared;
extern volatile uint32_t g_checkin[WDT_COUNT];
extern volatile int g_fault_requested;

static inline void app_checkin(int idx)
{
    g_checkin[idx]++;
}

/* Task entry points. */
void sensor_task(void *arg);
void fusion_task(void *arg);
void state_machine_task(void *arg);
void telemetry_task(void *arg);
void watchdog_task(void *arg);

/* Terminate the QEMU session via semihosting (used when the flight finishes
 * or on a fatal condition), so CI runs are bounded. */
void qemu_exit(int code);

#endif /* AEROPILOT_APP_H */
