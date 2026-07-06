#include "unity.h"

#include <math.h>

#include "aeropilot.h"
#include "fusion/complementary.h"

void setUp(void) {}
void tearDown(void) {}

static void test_vertical_init_seeds_from_baro(void)
{
    comp_vertical_t c;
    comp_vertical_init(&c, 0.98f);
    comp_vertical_step(&c, 0.0f, 15.0f, 0.01f);
    TEST_ASSERT_EQUAL_FLOAT(15.0f, c.altitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, c.velocity);
}

static void test_vertical_tracks_ramp(void)
{
    comp_vertical_t c;
    comp_vertical_init(&c, 0.9f);
    const float dt = 0.01f;
    const float v = 10.0f;
    float t = 0.0f;
    for (int i = 0; i < 500; ++i)
    {
        t += dt;
        comp_vertical_step(&c, 0.0f, v * t, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(2.0f, v * t, c.altitude);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, v, c.velocity);
}

static void test_attitude_level_when_gravity_only(void)
{
    comp_attitude_t a;
    comp_attitude_init(&a, 0.98f);
    for (int i = 0; i < 300; ++i)
    {
        comp_attitude_step(&a, 0.0f, 0.0f, 0.0f, 0.0f,
                           AEROPILOT_GRAVITY_MS2, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, a.pitch);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.0f, a.roll);
}

static void test_attitude_gyro_integration(void)
{
    comp_attitude_t a;
    comp_attitude_init(&a, 1.0f); /* pure gyro integration */
    const float rate = 0.5f;      /* rad/s pitch */
    const float dt = 0.01f;
    for (int i = 0; i < 100; ++i)
    {
        comp_attitude_step(&a, 0.0f, rate, 0.0f, 0.0f,
                           AEROPILOT_GRAVITY_MS2, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.02f, rate * 1.0f, a.pitch);
}

static void test_attitude_converges_to_tilt(void)
{
    comp_attitude_t a;
    comp_attitude_init(&a, 0.9f);
    /* Hold a static ~30 deg roll: ay/az reflect gravity components. */
    float roll = 0.5236f; /* 30 deg */
    float ay = AEROPILOT_GRAVITY_MS2 * sinf(roll);
    float az = AEROPILOT_GRAVITY_MS2 * cosf(roll);
    for (int i = 0; i < 400; ++i)
    {
        comp_attitude_step(&a, 0.0f, 0.0f, 0.0f, ay, az, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.05f, roll, a.roll);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_vertical_init_seeds_from_baro);
    RUN_TEST(test_vertical_tracks_ramp);
    RUN_TEST(test_attitude_level_when_gravity_only);
    RUN_TEST(test_attitude_gyro_integration);
    RUN_TEST(test_attitude_converges_to_tilt);
    return UNITY_END();
}
