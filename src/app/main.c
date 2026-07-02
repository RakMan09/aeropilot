#include <stdio.h>

#include "app/app.h"
#include "mps2_uart.h"

QueueHandle_t g_sensor_queue;
QueueHandle_t g_estimate_queue;
shared_state_t g_shared;
volatile uint32_t g_checkin[WDT_COUNT];
volatile int g_fault_requested;
volatile uint32_t g_jitter_samples;
volatile uint32_t g_period_min_cyc;
volatile uint32_t g_period_max_cyc;
volatile uint32_t g_jitter_max_cyc;

void qemu_exit(int code)
{
    (void)code;
    /* ARM semihosting SYS_EXIT (angel_SWIreason_ReportException) with
     * ADP_Stopped_ApplicationExit. QEMU terminates when run with -semihosting.
     */
    register uint32_t r0 __asm__("r0") = 0x18u;         /* SYS_EXIT */
    register uint32_t r1 __asm__("r1") = 0x20026u;      /* ApplicationExit */
    __asm__ volatile("bkpt 0xAB" : : "r"(r0), "r"(r1) : "memory");
    for (;;)
    {
    }
}

int main(void)
{
    uart_init(MPS2_UART0_BASE); /* log console (stdout) */
    uart_init(MPS2_UART1_BASE); /* telemetry bus link */

    printf("\n=== AeroPilot flight computer (FreeRTOS/QEMU mps2-an385) ===\n");

    g_shared.lock = xSemaphoreCreateMutex();
    g_sensor_queue = xQueueCreate(8, sizeof(sensor_sample_t));
    g_estimate_queue = xQueueCreate(8, sizeof(estimate_msg_t));

    if (g_shared.lock == NULL || g_sensor_queue == NULL ||
        g_estimate_queue == NULL)
    {
        printf("[main] allocation failure\n");
        qemu_exit(1);
    }

    g_shared.state = FLIGHT_IDLE;

    (void)xTaskCreate(watchdog_task, "wd", configMINIMAL_STACK_SIZE * 2,
                      NULL, PRIO_WATCHDOG, NULL);
    (void)xTaskCreate(sensor_task, "sensor", configMINIMAL_STACK_SIZE * 4,
                      NULL, PRIO_SENSOR, NULL);
    (void)xTaskCreate(fusion_task, "fusion", configMINIMAL_STACK_SIZE * 4,
                      NULL, PRIO_FUSION, NULL);
    (void)xTaskCreate(state_machine_task, "sm", configMINIMAL_STACK_SIZE * 4,
                      NULL, PRIO_STATE, NULL);
    (void)xTaskCreate(telemetry_task, "tele", configMINIMAL_STACK_SIZE * 4,
                      NULL, PRIO_TELEMETRY, NULL);

    vTaskStartScheduler();

    /* Only reached if the scheduler could not start. */
    for (;;)
    {
    }
    return 0;
}

/* ----- FreeRTOS hooks ----- */

void vApplicationMallocFailedHook(void)
{
    printf("[main] malloc failed\n");
    qemu_exit(1);
}

void vApplicationStackOverflowHook(TaskHandle_t task, char *name)
{
    (void)task;
    printf("[main] stack overflow in %s\n", name);
    qemu_exit(1);
}

/* Static memory for the idle and timer tasks (configSUPPORT_STATIC_ALLOCATION).
 */
void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
                                   StackType_t **ppxIdleTaskStackBuffer,
                                   uint32_t *pulIdleTaskStackSize)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];
    *ppxIdleTaskTCBBuffer = &idle_tcb;
    *ppxIdleTaskStackBuffer = idle_stack;
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

void vApplicationGetTimerTaskMemory(StaticTask_t **ppxTimerTaskTCBBuffer,
                                    StackType_t **ppxTimerTaskStackBuffer,
                                    uint32_t *pulTimerTaskStackSize)
{
    static StaticTask_t timer_tcb;
    static StackType_t timer_stack[configTIMER_TASK_STACK_DEPTH];
    *ppxTimerTaskTCBBuffer = &timer_tcb;
    *ppxTimerTaskStackBuffer = timer_stack;
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
