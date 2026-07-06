#include "unity.h"

#include <math.h>

#include "aeropilot.h"
#include "sim/rocket_sim.h"

void setUp(void) {}
void tearDown(void) {}

static rocket_sim_params_t clean_params(void)
{
    rocket_sim_params_t p;
    rocket_sim_default_params(&p);
    p.accel_noise = 0.0f;
    p.baro_noise = 0.0f;
    return p;
}

static void test_starts_on_pad(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.altitude);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, sim.velocity);
    TEST_ASSERT_FALSE(rocket_sim_landed(&sim));
}

static void test_accelerometer_reads_gravity_at_rest(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    sensor_sample_t s = rocket_sim_read(&sim);
    /* At rest the accelerometer measures +1 g of specific force. */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, AEROPILOT_GRAVITY_MS2, s.az);
}

static void test_ground_pressure_is_baseline(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    sensor_sample_t s = rocket_sim_read(&sim);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, p.ground_pressure, s.pressure);
}

static void test_climbs_during_boost(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    for (int i = 0; i < 100; ++i) /* 1 s at dt=0.01 */
    {
        rocket_sim_advance(&sim, 0.01f);
    }
    TEST_ASSERT_TRUE(sim.altitude > 5.0f);
    TEST_ASSERT_TRUE(sim.velocity > 0.0f);
}

static void test_reaches_and_returns(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    for (int i = 0; i < 6000 && !rocket_sim_landed(&sim); ++i)
    {
        rocket_sim_advance(&sim, 0.01f);
    }
    TEST_ASSERT_TRUE(rocket_sim_landed(&sim));
}

static void test_true_apogee_reasonable(void)
{
    rocket_sim_params_t p = clean_params();
    float apogee = 0.0f, at = 0.0f;
    rocket_sim_true_apogee(&p, &apogee, &at);
    TEST_ASSERT_TRUE(apogee > 50.0f);
    TEST_ASSERT_TRUE(apogee < 1000.0f);
    TEST_ASSERT_TRUE(at > 0.0f);
}

static void test_pressure_drops_with_altitude(void)
{
    rocket_sim_params_t p = clean_params();
    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);
    sensor_sample_t ground = rocket_sim_read(&sim);
    for (int i = 0; i < 150; ++i)
    {
        rocket_sim_advance(&sim, 0.01f);
    }
    sensor_sample_t up = rocket_sim_read(&sim);
    TEST_ASSERT_TRUE(up.pressure < ground.pressure);
}

static void test_deterministic_with_seed(void)
{
    rocket_sim_params_t p;
    rocket_sim_default_params(&p); /* with noise */
    rocket_sim_t a, b;
    rocket_sim_init(&a, &p);
    rocket_sim_init(&b, &p);
    for (int i = 0; i < 300; ++i)
    {
        rocket_sim_advance(&a, 0.01f);
        rocket_sim_advance(&b, 0.01f);
        sensor_sample_t sa = rocket_sim_read(&a);
        sensor_sample_t sb = rocket_sim_read(&b);
        TEST_ASSERT_EQUAL_FLOAT(sa.az, sb.az);
        TEST_ASSERT_EQUAL_FLOAT(sa.pressure, sb.pressure);
    }
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_on_pad);
    RUN_TEST(test_accelerometer_reads_gravity_at_rest);
    RUN_TEST(test_ground_pressure_is_baseline);
    RUN_TEST(test_climbs_during_boost);
    RUN_TEST(test_reaches_and_returns);
    RUN_TEST(test_true_apogee_reasonable);
    RUN_TEST(test_pressure_drops_with_altitude);
    RUN_TEST(test_deterministic_with_seed);
    return UNITY_END();
}
