#include "unity.h"

#include <string.h>

#include "proto/bus.h"
#include "proto/frame.h"

void setUp(void) {}
void tearDown(void) {}

static void build_frame(uint8_t *frame)
{
    telemetry_t t;
    t.seq = 42;
    t.state = 4;
    t.flags = 0;
    t.altitude = 250.75f;
    t.velocity = 12.5f;
    t.batt_mv = 8000;
    telemetry_frame_encode(&t, frame);
}

static void test_wire_size(void)
{
    uint8_t frame[TELEMETRY_FRAME_SIZE];
    build_frame(frame);
    uint8_t wire[BUS_WIRE_BYTES_PER_FRAME(TELEMETRY_FRAME_SIZE)];
    size_t n = bus_encode_frame(frame, sizeof(frame), wire, sizeof(wire));
    TEST_ASSERT_EQUAL_UINT(BUS_WIRE_BYTES_PER_FRAME(TELEMETRY_FRAME_SIZE), n);
    TEST_ASSERT_EQUAL_UINT(36u, n); /* 18 bytes -> 9 words -> 36 bytes */
}

static void test_odd_length_rejected(void)
{
    uint8_t frame[3] = {1, 2, 3};
    uint8_t wire[16];
    TEST_ASSERT_EQUAL_UINT(0u, bus_encode_frame(frame, 3, wire, sizeof(wire)));
}

static void test_insufficient_capacity(void)
{
    uint8_t frame[TELEMETRY_FRAME_SIZE];
    build_frame(frame);
    uint8_t wire[8];
    TEST_ASSERT_EQUAL_UINT(0u, bus_encode_frame(frame, sizeof(frame),
                                                wire, sizeof(wire)));
}

static void test_encode_decode_round_trip(void)
{
    uint8_t frame[TELEMETRY_FRAME_SIZE];
    build_frame(frame);
    uint8_t wire[BUS_WIRE_BYTES_PER_FRAME(TELEMETRY_FRAME_SIZE)];
    size_t n = bus_encode_frame(frame, sizeof(frame), wire, sizeof(wire));

    uint8_t recovered[TELEMETRY_FRAME_SIZE];
    int parity_errors = 0;
    size_t got = bus_decode_words(wire, n, recovered, sizeof(recovered),
                                  &parity_errors);
    TEST_ASSERT_EQUAL_UINT(TELEMETRY_FRAME_SIZE, got);
    TEST_ASSERT_EQUAL_INT(0, parity_errors);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(frame, recovered, TELEMETRY_FRAME_SIZE);

    /* And the recovered frame still validates end-to-end. */
    telemetry_t out;
    TEST_ASSERT_EQUAL_INT(0, telemetry_frame_decode(recovered, &out));
    TEST_ASSERT_EQUAL_FLOAT(250.75f, out.altitude);
}

static void test_word_corruption_counted(void)
{
    uint8_t frame[TELEMETRY_FRAME_SIZE];
    build_frame(frame);
    uint8_t wire[BUS_WIRE_BYTES_PER_FRAME(TELEMETRY_FRAME_SIZE)];
    size_t n = bus_encode_frame(frame, sizeof(frame), wire, sizeof(wire));

    wire[0] ^= 0x01; /* corrupt first word's parity */

    uint8_t recovered[TELEMETRY_FRAME_SIZE];
    int parity_errors = 0;
    size_t got = bus_decode_words(wire, n, recovered, sizeof(recovered),
                                  &parity_errors);
    TEST_ASSERT_EQUAL_INT(1, parity_errors);
    TEST_ASSERT_EQUAL_UINT(TELEMETRY_FRAME_SIZE - 2u, got);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_wire_size);
    RUN_TEST(test_odd_length_rejected);
    RUN_TEST(test_insufficient_capacity);
    RUN_TEST(test_encode_decode_round_trip);
    RUN_TEST(test_word_corruption_counted);
    return UNITY_END();
}
