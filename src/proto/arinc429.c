#include "proto/arinc429.h"

#define ARINC429_PARITY_BIT (1u << 31)

static uint8_t reverse8(uint8_t v)
{
    v = (uint8_t)(((v & 0xF0u) >> 4) | ((v & 0x0Fu) << 4));
    v = (uint8_t)(((v & 0xCCu) >> 2) | ((v & 0x33u) << 2));
    v = (uint8_t)(((v & 0xAAu) >> 1) | ((v & 0x55u) << 1));
    return v;
}

static int popcount32(uint32_t v)
{
    int count = 0;
    while (v != 0u)
    {
        v &= (v - 1u);
        ++count;
    }
    return count;
}

uint32_t arinc429_encode(uint8_t label, uint8_t sdi, uint32_t data19,
                         uint8_t ssm)
{
    uint32_t word = 0u;

    word |= (uint32_t)reverse8(label);
    word |= ((uint32_t)(sdi & 0x3u)) << 8;
    word |= (data19 & 0x7FFFFu) << 10;
    word |= ((uint32_t)(ssm & 0x3u)) << 29;

    /* Odd parity across the whole word. */
    if ((popcount32(word) & 1) == 0)
    {
        word |= ARINC429_PARITY_BIT;
    }

    return word;
}

int arinc429_parity_ok(uint32_t word)
{
    return (popcount32(word) & 1) == 1;
}

uint8_t arinc429_label(uint32_t word)
{
    return reverse8((uint8_t)(word & 0xFFu));
}

uint8_t arinc429_sdi(uint32_t word)
{
    return (uint8_t)((word >> 8) & 0x3u);
}

uint32_t arinc429_data(uint32_t word)
{
    return (word >> 10) & 0x7FFFFu;
}

uint8_t arinc429_ssm(uint32_t word)
{
    return (uint8_t)((word >> 29) & 0x3u);
}

uint32_t arinc429_encode_payload(uint16_t two_bytes)
{
    return arinc429_encode((uint8_t)ARINC429_LABEL_TELEMETRY, 0u,
                           (uint32_t)two_bytes, 0u);
}

int arinc429_decode_payload(uint32_t word, uint16_t *two_bytes)
{
    if (!arinc429_parity_ok(word))
    {
        return -1;
    }
    if (arinc429_label(word) != (uint8_t)ARINC429_LABEL_TELEMETRY)
    {
        return -2;
    }
    *two_bytes = (uint16_t)(arinc429_data(word) & 0xFFFFu);
    return 0;
}
