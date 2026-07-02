#include "proto/frame.h"

#include <string.h>

#include "proto/crc16.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFu);
    p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_f32(uint8_t *p, float v)
{
    uint32_t bits;
    memcpy(&bits, &v, sizeof(bits));
    p[0] = (uint8_t)(bits & 0xFFu);
    p[1] = (uint8_t)((bits >> 8) & 0xFFu);
    p[2] = (uint8_t)((bits >> 16) & 0xFFu);
    p[3] = (uint8_t)((bits >> 24) & 0xFFu);
}

static float get_f32(const uint8_t *p)
{
    uint32_t bits = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
                    ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
    float v;
    memcpy(&v, &bits, sizeof(v));
    return v;
}

size_t telemetry_frame_encode(const telemetry_t *tel, uint8_t *out)
{
    put_u16(&out[0], (uint16_t)TELEMETRY_SYNC);
    put_u16(&out[2], tel->seq);
    out[4] = tel->state;
    out[5] = tel->flags;
    put_f32(&out[6], tel->altitude);
    put_f32(&out[10], tel->velocity);
    put_u16(&out[14], tel->batt_mv);

    uint16_t crc = crc16_ccitt(out, 16u);
    put_u16(&out[16], crc);

    return (size_t)TELEMETRY_FRAME_SIZE;
}

int telemetry_frame_decode(const uint8_t *in, telemetry_t *tel)
{
    if (get_u16(&in[0]) != (uint16_t)TELEMETRY_SYNC)
    {
        return -1;
    }

    uint16_t crc = crc16_ccitt(in, 16u);
    if (crc != get_u16(&in[16]))
    {
        return -2;
    }

    tel->seq = get_u16(&in[2]);
    tel->state = in[4];
    tel->flags = in[5];
    tel->altitude = get_f32(&in[6]);
    tel->velocity = get_f32(&in[10]);
    tel->batt_mv = get_u16(&in[14]);
    return 0;
}
