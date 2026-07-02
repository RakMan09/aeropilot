#include "unity.h"

#include "proto/arinc429.h"

void setUp(void) {}
void tearDown(void) {}

static void test_parity_is_odd(void)
{
    for (uint32_t d = 0; d < 4096u; ++d)
    {
        uint32_t w = arinc429_encode(0311u, 0u, d, 0u);
        TEST_ASSERT_TRUE(arinc429_parity_ok(w));
    }
}

static void test_label_round_trip(void)
{
    uint32_t w = arinc429_encode(0311u, 0u, 0x5555u, 0u);
    TEST_ASSERT_EQUAL_UINT8(0311u, arinc429_label(w));
}

static void test_various_labels(void)
{
    uint8_t labels[] = {0u, 1u, 0311u, 0377u, 0125u, 0252u};
    for (unsigned i = 0; i < sizeof(labels); ++i)
    {
        uint32_t w = arinc429_encode(labels[i], 0u, 0x1234u, 0u);
        TEST_ASSERT_EQUAL_UINT8(labels[i], arinc429_label(w));
        TEST_ASSERT_TRUE(arinc429_parity_ok(w));
    }
}

static void test_field_extraction(void)
{
    uint32_t w = arinc429_encode(0311u, 0x3u, 0x7ABCDu, 0x2u);
    TEST_ASSERT_EQUAL_UINT8(0x3u, arinc429_sdi(w));
    TEST_ASSERT_EQUAL_HEX32(0x7ABCDu, arinc429_data(w));
    TEST_ASSERT_EQUAL_UINT8(0x2u, arinc429_ssm(w));
}

static void test_data_masked_to_19_bits(void)
{
    uint32_t w = arinc429_encode(0311u, 0u, 0xFFFFFFFFu, 0u);
    TEST_ASSERT_EQUAL_HEX32(0x7FFFFu, arinc429_data(w));
}

static void test_payload_round_trip(void)
{
    for (uint32_t v = 0; v <= 0xFFFFu; v += 137u)
    {
        uint32_t w = arinc429_encode_payload((uint16_t)v);
        uint16_t out = 0;
        TEST_ASSERT_EQUAL_INT(0, arinc429_decode_payload(w, &out));
        TEST_ASSERT_EQUAL_UINT16((uint16_t)v, out);
    }
}

static void test_parity_error_rejected(void)
{
    uint32_t w = arinc429_encode_payload(0xBEEFu);
    w ^= 0x00000010u; /* flip a data bit -> parity now wrong */
    uint16_t out;
    TEST_ASSERT_EQUAL_INT(-1, arinc429_decode_payload(w, &out));
}

static void test_wrong_label_rejected(void)
{
    /* Encode with a non-telemetry label but valid parity. */
    uint32_t w = arinc429_encode(0123u, 0u, 0xABCDu, 0u);
    uint16_t out;
    TEST_ASSERT_EQUAL_INT(-2, arinc429_decode_payload(w, &out));
}

static void test_every_single_bit_flip_detected(void)
{
    uint32_t w = arinc429_encode_payload(0x1357u);
    for (int bit = 0; bit < 32; ++bit)
    {
        uint32_t c = w ^ (1u << bit);
        uint16_t out;
        int rc = arinc429_decode_payload(c, &out);
        /* A single bit flip breaks odd parity, or changes the label, so the
         * word must be rejected (or, if it flips a data bit within a valid
         * label+parity combo, parity guarantees rejection). */
        TEST_ASSERT_NOT_EQUAL(0, rc);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_parity_is_odd);
    RUN_TEST(test_label_round_trip);
    RUN_TEST(test_various_labels);
    RUN_TEST(test_field_extraction);
    RUN_TEST(test_data_masked_to_19_bits);
    RUN_TEST(test_payload_round_trip);
    RUN_TEST(test_parity_error_rejected);
    RUN_TEST(test_wrong_label_rejected);
    RUN_TEST(test_every_single_bit_flip_detected);
    return UNITY_END();
}
