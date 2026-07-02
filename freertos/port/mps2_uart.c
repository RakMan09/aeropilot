#include "mps2_uart.h"

/* CMSDK APB UART register layout (offsets from the peripheral base). */
typedef volatile struct
{
    uint32_t DATA;   /* 0x00: RX/TX data. */
    uint32_t STATE;  /* 0x04: bit0 = TX buffer full, bit1 = RX buffer full. */
    uint32_t CTRL;   /* 0x08: bit0 = TX enable, bit1 = RX enable. */
    uint32_t INT;    /* 0x0C: interrupt status/clear. */
    uint32_t BAUDDIV;/* 0x10: baud rate divider. */
} cmsdk_uart_t;

#define UART_STATE_TX_FULL (1U << 0)
#define UART_CTRL_TX_EN    (1U << 0)

static cmsdk_uart_t *uart_at(uint32_t base)
{
    return (cmsdk_uart_t *)base;
}

void uart_init(uint32_t base)
{
    cmsdk_uart_t *uart = uart_at(base);

    /* QEMU ignores the actual baud value, but a non-zero divider keeps the
     * model happy and mirrors real hardware bring-up. */
    uart->BAUDDIV = 16U;
    uart->CTRL = UART_CTRL_TX_EN;
}

void uart_putc(uint32_t base, char c)
{
    cmsdk_uart_t *uart = uart_at(base);

    while ((uart->STATE & UART_STATE_TX_FULL) != 0U)
    {
    }

    uart->DATA = (uint32_t)(uint8_t)c;
}

void uart_write(uint32_t base, const uint8_t *data, size_t len)
{
    for (size_t i = 0U; i < len; ++i)
    {
        uart_putc(base, (char)data[i]);
    }
}
