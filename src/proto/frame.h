/* Fixed-size telemetry frame: the on-the-wire unit carried by the simulated
 * avionics bus.
 *
 * Layout (little-endian, 18 bytes total):
 *   off  size  field
 *   0    2     sync   = 0xA55A
 *   2    2     seq    (uint16, wraps)
 *   4    1     state  (flight_state_t)
 *   5    1     flags  (bit0 deployed, bit1 safe/fault)
 *   6    4     alt    (float32, m)
 *   10   4     vel    (float32, m/s)
 *   14   2     batt   (uint16, millivolts)
 *   16   2     crc16  (CRC-16/CCITT over bytes 0..15)
 *
 * Serialisation is done byte-by-byte so the format is independent of host
 * struct packing and endianness. Pure C11, no RTOS dependency. */
#ifndef AEROPILOT_FRAME_H
#define AEROPILOT_FRAME_H

#include <stddef.h>
#include <stdint.h>

#define TELEMETRY_SYNC       0xA55Au
#define TELEMETRY_FRAME_SIZE 18u

#define TELEMETRY_FLAG_DEPLOYED (1u << 0)
#define TELEMETRY_FLAG_SAFE     (1u << 1)

typedef struct
{
    uint16_t seq;
    uint8_t  state;
    uint8_t  flags;
    float    altitude;
    float    velocity;
    uint16_t batt_mv;
} telemetry_t;

/* Serialise tel into out (must hold TELEMETRY_FRAME_SIZE bytes), appending the
 * sync word and CRC. Returns the number of bytes written. */
size_t telemetry_frame_encode(const telemetry_t *tel, uint8_t *out);

/* Parse a TELEMETRY_FRAME_SIZE-byte buffer. Returns 0 on success, or a
 * negative value if the sync word or CRC is invalid. */
int telemetry_frame_decode(const uint8_t *in, telemetry_t *tel);

#endif /* AEROPILOT_FRAME_H */
