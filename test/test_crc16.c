#include "unity.h"

#include <string.h>

#include "proto/crc16.h"

void setUp(void) {}
void tearDown(void) {}

/* CRC-16/CCITT-FALSE check value: "123456789" -> 0x29B1. */
static void test_check_value(void)
{
    const uint8_t msg[] = "123456789";
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc16_ccitt(msg, 9));
}

static void test_empty(void)
{
    TEST_ASSERT_EQUAL_HEX16(0xFFFF, crc16_ccitt((const uint8_t *)"", 0));
}

static void test_single_bit_change_detected(void)
{
    uint8_t a[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    uint8_t b[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    b[3] ^= 0x01;
    TEST_ASSERT_NOT_EQUAL(crc16_ccitt(a, 8), crc16_ccitt(b, 8));
}

static void test_deterministic(void)
{
    uint8_t data[16];
    for (int i = 0; i < 16; ++i)
    {
        data[i] = (uint8_t)(i * 7 + 3);
    }
    TEST_ASSERT_EQUAL_HEX16(crc16_ccitt(data, 16), crc16_ccitt(data, 16));
}

static void test_length_sensitivity(void)
{
    uint8_t data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22};
    TEST_ASSERT_NOT_EQUAL(crc16_ccitt(data, 7), crc16_ccitt(data, 8));
}

static void test_all_zeros(void)
{
    uint8_t z[10] = {0};
    /* Known: CRC of 10 zero bytes with 0xFFFF init is stable and non-zero. */
    uint16_t crc = crc16_ccitt(z, 10);
    TEST_ASSERT_NOT_EQUAL(0x0000, crc);
    TEST_ASSERT_EQUAL_HEX16(crc, crc16_ccitt(z, 10));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_check_value);
    RUN_TEST(test_empty);
    RUN_TEST(test_single_bit_change_detected);
    RUN_TEST(test_deterministic);
    RUN_TEST(test_length_sensitivity);
    RUN_TEST(test_all_zeros);
    return UNITY_END();
}
