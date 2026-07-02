/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection, xorout 0).
 * Used to protect telemetry frames on the simulated avionics bus. */
#ifndef AEROPILOT_CRC16_H
#define AEROPILOT_CRC16_H

#include <stddef.h>
#include <stdint.h>

uint16_t crc16_ccitt(const uint8_t *data, size_t len);

#endif /* AEROPILOT_CRC16_H */
