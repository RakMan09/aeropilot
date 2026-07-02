/* Minimal Cortex-M3 startup for the QEMU mps2-an385 target.
 *
 * Provides the vector table, reset handler (C runtime init) and the default
 * fault handlers. The SVC/PendSV/SysTick vectors are routed to the FreeRTOS
 * ARM_CM3 port handlers.
 */

#include <stdint.h>

extern uint32_t _sidata;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t __StackTop;

extern int main(void);

/* FreeRTOS ARM_CM3 port handlers. */
extern void vPortSVCHandler(void);
extern void xPortPendSVHandler(void);
extern void xPortSysTickHandler(void);

void Reset_Handler(void);
void Default_Handler(void);

void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void MemManage_Handler(void) __attribute__((weak, alias("Default_Handler")));
void BusFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void UsageFault_Handler(void) __attribute__((weak, alias("Default_Handler")));

/* The vector table. Only the handlers we actually use are populated. */
__attribute__((section(".isr_vector"), used))
void (*const g_vector_table[])(void) = {
    (void (*)(void))(&__StackTop), /* Initial stack pointer. */
    Reset_Handler,                 /* Reset. */
    NMI_Handler,
    HardFault_Handler,
    MemManage_Handler,
    BusFault_Handler,
    UsageFault_Handler,
    0, 0, 0, 0,                    /* Reserved. */
    vPortSVCHandler,               /* SVCall. */
    0,                             /* Debug monitor. */
    0,                             /* Reserved. */
    xPortPendSVHandler,            /* PendSV. */
    xPortSysTickHandler,           /* SysTick. */
};

void Reset_Handler(void)
{
    uint32_t *src = &_sidata;
    uint32_t *dst = &_sdata;

    while (dst < &_edata)
    {
        *dst++ = *src++;
    }

    for (dst = &_sbss; dst < &_ebss;)
    {
        *dst++ = 0U;
    }

    (void)main();

    for (;;)
    {
    }
}

void Default_Handler(void)
{
    for (;;)
    {
    }
}
