/* Driver for the ARM CMSDK APB UART peripherals present on the MPS2 AN385. */
#ifndef AEROPILOT_MPS2_UART_H
#define AEROPILOT_MPS2_UART_H

#include <stddef.h>
#include <stdint.h>

/* UART peripheral base addresses on the MPS2 AN385 memory map. */
#define MPS2_UART0_BASE 0x40004000UL /* Human-readable log console. */
#define MPS2_UART1_BASE 0x40005000UL /* Avionics telemetry bus link. */

void uart_init(uint32_t base);
void uart_putc(uint32_t base, char c);
void uart_write(uint32_t base, const uint8_t *data, size_t len);

#endif /* AEROPILOT_MPS2_UART_H */
