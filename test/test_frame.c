#include "unity.h"

#include <string.h>

#include "proto/frame.h"

void setUp(void) {}
void tearDown(void) {}

static telemetry_t sample_tel(void)
{
    telemetry_t t;
    t.seq = 0x1234;
    t.state = 3;
    t.flags = TELEMETRY_FLAG_DEPLOYED;
    t.altitude = 123.5f;
    t.velocity = -4.25f;
    t.batt_mv = 8123;
    return t;
}

static void test_encode_size(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    TEST_ASSERT_EQUAL_UINT(TELEMETRY_FRAME_SIZE,
                           telemetry_frame_encode(&t, buf));
}

static void test_sync_word(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);
    /* Little-endian sync. */
    TEST_ASSERT_EQUAL_HEX8(0x5A, buf[0]);
    TEST_ASSERT_EQUAL_HEX8(0xA5, buf[1]);
}

static void test_round_trip(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);

    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(0, telemetry_frame_decode(buf, &out));
    TEST_ASSERT_EQUAL_UINT16(t.seq, out.seq);
    TEST_ASSERT_EQUAL_UINT8(t.state, out.state);
    TEST_ASSERT_EQUAL_UINT8(t.flags, out.flags);
    TEST_ASSERT_EQUAL_FLOAT(t.altitude, out.altitude);
    TEST_ASSERT_EQUAL_FLOAT(t.velocity, out.velocity);
    TEST_ASSERT_EQUAL_UINT16(t.batt_mv, out.batt_mv);
}

static void test_bad_sync_rejected(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);
    buf[0] ^= 0xFF;
    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(-1, telemetry_frame_decode(buf, &out));
}

static void test_bad_crc_rejected(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);
    buf[8] ^= 0x08; /* corrupt a payload byte, leave sync intact */
    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(-2, telemetry_frame_decode(buf, &out));
}

static void test_every_payload_bitflip_detected(void)
{
    telemetry_t t = sample_tel();
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);

    /* Flip each bit of the payload region [2..15] and confirm CRC catches it. */
    for (int byte = 2; byte < 16; ++byte)
    {
        for (int bit = 0; bit < 8; ++bit)
        {
            uint8_t corrupt[TELEMETRY_FRAME_SIZE];
            memcpy(corrupt, buf, sizeof(corrupt));
            corrupt[byte] ^= (uint8_t)(1u << bit);
            telemetry_t out;
            TEST_ASSERT_EQUAL_INT(-2, telemetry_frame_decode(corrupt, &out));
        }
    }
}

static void test_negative_and_zero_values(void)
{
    telemetry_t t = sample_tel();
    t.altitude = 0.0f;
    t.velocity = 0.0f;
    t.seq = 0;
    t.flags = 0;
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);
    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(0, telemetry_frame_decode(buf, &out));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, out.altitude);
    TEST_ASSERT_EQUAL_UINT16(0, out.seq);
}

static void test_seq_wrap(void)
{
    telemetry_t t = sample_tel();
    t.seq = 0xFFFF;
    uint8_t buf[TELEMETRY_FRAME_SIZE];
    telemetry_frame_encode(&t, buf);
    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(0, telemetry_frame_decode(buf, &out));
    TEST_ASSERT_EQUAL_UINT16(0xFFFF, out.seq);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_encode_size);
    RUN_TEST(test_sync_word);
    RUN_TEST(test_round_trip);
    RUN_TEST(test_bad_sync_rejected);
    RUN_TEST(test_bad_crc_rejected);
    RUN_TEST(test_every_payload_bitflip_detected);
    RUN_TEST(test_negative_and_zero_values);
    RUN_TEST(test_seq_wrap);
    return UNITY_END();
}
