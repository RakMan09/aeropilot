#include <stdio.h>

#include "app/app.h"
#include "mps2_uart.h"
#include "proto/bus.h"
#include "proto/frame.h"

/* Periodically snapshots the shared vehicle state, serialises it into a
 * CRC-protected telemetry frame, packs it into ARINC-429 words, and streams
 * the words over UART1 (the simulated avionics bus link). */
void telemetry_task(void *arg)
{
    (void)arg;

    uint16_t seq = 0u;
    uint16_t batt_mv = 8400u; /* 2S LiPo, drains slowly over the flight. */

    TickType_t last_wake = xTaskGetTickCount();

    for (;;)
    {
        telemetry_t tel;
        tel.seq = seq++;
        tel.batt_mv = batt_mv;
        if (batt_mv > 7000u)
        {
            batt_mv -= 1u;
        }

        if (xSemaphoreTake(g_shared.lock, portMAX_DELAY) == pdTRUE)
        {
            tel.state = (uint8_t)g_shared.state;
            tel.altitude = g_shared.est.altitude;
            tel.velocity = g_shared.est.velocity;
            tel.flags = 0u;
            if (g_shared.deployed)
            {
                tel.flags |= TELEMETRY_FLAG_DEPLOYED;
            }
            if (g_shared.safe)
            {
                tel.flags |= TELEMETRY_FLAG_SAFE;
            }
            (void)xSemaphoreGive(g_shared.lock);
        }

        uint8_t frame[TELEMETRY_FRAME_SIZE];
        (void)telemetry_frame_encode(&tel, frame);

        uint8_t wire[BUS_WIRE_BYTES_PER_FRAME(TELEMETRY_FRAME_SIZE)];
        size_t n = bus_encode_frame(frame, sizeof(frame), wire, sizeof(wire));

        uart_write(MPS2_UART1_BASE, wire, n);

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TELEMETRY_PERIOD_MS));
    }
}
