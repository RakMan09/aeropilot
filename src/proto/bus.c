#include "proto/bus.h"

#include "proto/arinc429.h"

static void put_word_le(uint8_t *p, uint32_t word)
{
    p[0] = (uint8_t)(word & 0xFFu);
    p[1] = (uint8_t)((word >> 8) & 0xFFu);
    p[2] = (uint8_t)((word >> 16) & 0xFFu);
    p[3] = (uint8_t)((word >> 24) & 0xFFu);
}

static uint32_t get_word_le(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

size_t bus_encode_frame(const uint8_t *frame, size_t frame_len,
                        uint8_t *out, size_t out_cap)
{
    if ((frame_len % 2u) != 0u)
    {
        return 0u;
    }

    size_t needed = BUS_WIRE_BYTES_PER_FRAME(frame_len);
    if (out_cap < needed)
    {
        return 0u;
    }

    size_t o = 0u;
    for (size_t i = 0u; i < frame_len; i += 2u)
    {
        uint16_t pair = (uint16_t)((uint16_t)frame[i] |
                                   ((uint16_t)frame[i + 1u] << 8));
        uint32_t word = arinc429_encode_payload(pair);
        put_word_le(&out[o], word);
        o += 4u;
    }
    return o;
}

size_t bus_decode_words(const uint8_t *in, size_t in_len,
                        uint8_t *frame_out, size_t frame_cap,
                        int *parity_errors)
{
    size_t out = 0u;

    for (size_t i = 0u; (i + 4u) <= in_len; i += 4u)
    {
        uint32_t word = get_word_le(&in[i]);
        uint16_t pair;
        if (arinc429_decode_payload(word, &pair) != 0)
        {
            if (parity_errors != NULL)
            {
                ++(*parity_errors);
            }
            continue;
        }
        if ((out + 2u) > frame_cap)
        {
            break;
        }
        frame_out[out++] = (uint8_t)(pair & 0xFFu);
        frame_out[out++] = (uint8_t)((pair >> 8) & 0xFFu);
    }

    return out;
}
