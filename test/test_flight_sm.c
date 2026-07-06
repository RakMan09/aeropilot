#include "unity.h"

#include "aeropilot.h"
#include "state/flight_sm.h"

static flight_sm_t sm;
static flight_sm_config_t cfg;

void setUp(void)
{
    flight_sm_default_config(&cfg);
    flight_sm_init(&sm, &cfg);
}
void tearDown(void) {}

static flight_state_t step(float t, float alt, float vel, float a_vert)
{
    state_estimate_t est = {0};
    est.t = t;
    est.altitude = alt;
    est.velocity = vel;
    return flight_sm_update(&sm, &est, a_vert);
}

static void test_starts_idle(void)
{
    TEST_ASSERT_EQUAL_INT(FLIGHT_IDLE, sm.state);
}

static void test_idle_ignores_motion_until_armed(void)
{
    for (int i = 0; i < 10; ++i)
    {
        step(0.01f * i, 0.0f, 0.0f, 50.0f);
    }
    TEST_ASSERT_EQUAL_INT(FLIGHT_IDLE, sm.state);
}

static void test_arm(void)
{
    flight_sm_arm(&sm);
    TEST_ASSERT_EQUAL_INT(FLIGHT_ARMED, sm.state);
}

static void test_boost_requires_debounce(void)
{
    flight_sm_arm(&sm);
    step(0.01f, 0.0f, 1.0f, 50.0f); /* 1 sample above threshold */
    TEST_ASSERT_EQUAL_INT(FLIGHT_ARMED, sm.state);
    step(0.02f, 0.1f, 2.0f, 50.0f);
    step(0.03f, 0.2f, 3.0f, 50.0f); /* 3rd -> BOOST */
    TEST_ASSERT_EQUAL_INT(FLIGHT_BOOST, sm.state);
}

static void drive_to_coast(void)
{
    flight_sm_arm(&sm);
    for (int i = 0; i < 5; ++i)
    {
        step(0.01f * i, (float)i, 10.0f, 50.0f);
    }
    /* burnout: sustained non-positive accel */
    for (int i = 0; i < 5; ++i)
    {
        step(0.1f + 0.01f * i, 30.0f, 40.0f, -2.0f);
    }
}

static void test_reaches_coast(void)
{
    drive_to_coast();
    TEST_ASSERT_EQUAL_INT(FLIGHT_COAST, sm.state);
}

static void test_apogee_deploys(void)
{
    drive_to_coast();
    /* velocity crosses zero at high altitude, debounced */
    for (int i = 0; i < 6; ++i)
    {
        step(1.0f + 0.01f * i, 100.0f, -0.1f, -9.8f);
    }
    TEST_ASSERT_EQUAL_INT(FLIGHT_APOGEE, sm.state);
    TEST_ASSERT_TRUE(flight_sm_deploy_fired(&sm));
}

static void test_no_early_apogee_below_min_altitude(void)
{
    drive_to_coast();
    /* velocity near zero but altitude below min_apogee_alt: must NOT deploy */
    for (int i = 0; i < 10; ++i)
    {
        step(1.0f + 0.01f * i, 5.0f, -0.1f, -9.8f);
    }
    TEST_ASSERT_FALSE(flight_sm_deploy_fired(&sm));
}

static void test_full_sequence_to_landed(void)
{
    drive_to_coast();
    for (int i = 0; i < 6; ++i)
    {
        step(1.0f + 0.01f * i, 100.0f, -0.1f, -9.8f);
    }
    /* descending clearly */
    step(1.2f, 90.0f, -20.0f, -9.8f);
    TEST_ASSERT_EQUAL_INT(FLIGHT_DESCENT, sm.state);
    /* landing: low altitude, low speed, debounced */
    for (int i = 0; i < 25; ++i)
    {
        step(2.0f + 0.01f * i, 0.5f, 0.0f, 0.0f);
    }
    TEST_ASSERT_EQUAL_INT(FLIGHT_LANDED, sm.state);
}

static void test_fault_latches_safe(void)
{
    flight_sm_arm(&sm);
    flight_sm_fault(&sm);
    TEST_ASSERT_EQUAL_INT(FLIGHT_SAFE, sm.state);
    /* subsequent updates stay in SAFE and never deploy */
    for (int i = 0; i < 20; ++i)
    {
        flight_state_t s = step(1.0f + 0.01f * i, 100.0f, -0.1f, -9.8f);
        TEST_ASSERT_EQUAL_INT(FLIGHT_SAFE, s);
    }
    TEST_ASSERT_FALSE(flight_sm_deploy_fired(&sm));
}

static void test_arm_ignored_after_fault(void)
{
    flight_sm_fault(&sm);
    flight_sm_arm(&sm);
    TEST_ASSERT_EQUAL_INT(FLIGHT_SAFE, sm.state);
}

static void test_max_altitude_tracked(void)
{
    flight_sm_arm(&sm);
    step(0.1f, 50.0f, 10.0f, 5.0f);
    step(0.2f, 120.0f, 5.0f, 1.0f);
    step(0.3f, 90.0f, -5.0f, -9.8f);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 120.0f, sm.max_altitude);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_starts_idle);
    RUN_TEST(test_idle_ignores_motion_until_armed);
    RUN_TEST(test_arm);
    RUN_TEST(test_boost_requires_debounce);
    RUN_TEST(test_reaches_coast);
    RUN_TEST(test_apogee_deploys);
    RUN_TEST(test_no_early_apogee_below_min_altitude);
    RUN_TEST(test_full_sequence_to_landed);
    RUN_TEST(test_fault_latches_safe);
    RUN_TEST(test_arm_ignored_after_fault);
    RUN_TEST(test_max_altitude_tracked);
    return UNITY_END();
}
