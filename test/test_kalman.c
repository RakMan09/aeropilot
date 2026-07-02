#include "unity.h"

#include <math.h>

#include "fusion/kalman.h"

void setUp(void) {}
void tearDown(void) {}

static void test_init_zero(void)
{
    kalman1d_t kf;
    kalman1d_init(&kf, 0.5f, 4.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kalman1d_altitude(&kf));
    TEST_ASSERT_EQUAL_FLOAT(0.0f, kalman1d_velocity(&kf));
}

static void test_stationary(void)
{
    kalman1d_t kf;
    kalman1d_init(&kf, 0.5f, 4.0f);
    for (int i = 0; i < 200; ++i)
    {
        kalman1d_step(&kf, 0.0f, 0.0f, 0.01f);
    }
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, kalman1d_altitude(&kf));
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, kalman1d_velocity(&kf));
}

static void test_tracks_constant_velocity(void)
{
    kalman1d_t kf;
    kalman1d_init(&kf, 0.5f, 1.0f);
    const float dt = 0.01f;
    const float v = 20.0f;
    float t = 0.0f;
    for (int i = 0; i < 500; ++i)
    {
        t += dt;
        kalman1d_step(&kf, 0.0f, v * t, dt);
    }
    TEST_ASSERT_FLOAT_WITHIN(2.0f, v * t, kalman1d_altitude(&kf));
    TEST_ASSERT_FLOAT_WITHIN(2.0f, v, kalman1d_velocity(&kf));
}

static void test_tracks_constant_acceleration(void)
{
    kalman1d_t kf;
    kalman1d_init(&kf, 1.0f, 1.0f);
    const float dt = 0.01f;
    const float a = 5.0f;
    float t = 0.0f;
    for (int i = 0; i < 400; ++i)
    {
        t += dt;
        float true_alt = 0.5f * a * t * t;
        kalman1d_step(&kf, a, true_alt, dt);
    }
    float true_alt = 0.5f * a * t * t;
    float true_vel = a * t;
    TEST_ASSERT_FLOAT_WITHIN(3.0f, true_alt, kalman1d_altitude(&kf));
    TEST_ASSERT_FLOAT_WITHIN(3.0f, true_vel, kalman1d_velocity(&kf));
}

static void test_rejects_measurement_noise(void)
{
    kalman1d_t kf;
    kalman1d_init(&kf, 0.01f, 25.0f); /* trust the model more than the baro */
    uint32_t rng = 12345u;
    for (int i = 0; i < 1000; ++i)
    {
        rng = rng * 1664525u + 1013904223u;
        float noise = (((float)(rng >> 8) / (float)0xFFFFFFu) - 0.5f) * 20.0f;
        kalman1d_step(&kf, 0.0f, 100.0f + noise, 0.01f);
    }
    /* Estimate should sit near the true constant altitude despite +/-10 m
     * measurement noise. */
    TEST_ASSERT_FLOAT_WITHIN(6.0f, 100.0f, kalman1d_altitude(&kf));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_zero);
    RUN_TEST(test_stationary);
    RUN_TEST(test_tracks_constant_velocity);
    RUN_TEST(test_tracks_constant_acceleration);
    RUN_TEST(test_rejects_measurement_noise);
    return UNITY_END();
}
