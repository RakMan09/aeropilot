#include "unity.h"

#include <math.h>

#include "aeropilot.h"
#include "fusion/fusion.h"
#include "sim/rocket_sim.h"

void setUp(void) {}
void tearDown(void) {}

/* Run the reference flight through a fusion mode and return the peak absolute
 * altitude error and the estimated apogee. */
static void run_flight(fusion_mode_t mode, float noise_scale,
                       float *max_err, float *est_apogee)
{
    rocket_sim_params_t p;
    rocket_sim_default_params(&p);
    p.accel_noise *= noise_scale;
    p.baro_noise *= noise_scale;

    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);

    fusion_t f;
    fusion_init(&f, mode, p.ground_pressure);

    float peak_err = 0.0f;
    float peak_est = 0.0f;

    for (int i = 0; i < 4000 && !rocket_sim_landed(&sim); ++i)
    {
        rocket_sim_advance(&sim, 0.01f);
        sensor_sample_t s = rocket_sim_read(&sim);
        state_estimate_t est = fusion_update(&f, &s);
        float err = fabsf(est.altitude - sim.altitude);
        if (err > peak_err)
        {
            peak_err = err;
        }
        if (est.altitude > peak_est)
        {
            peak_est = est.altitude;
        }
    }

    *max_err = peak_err;
    *est_apogee = peak_est;
}

static void test_kalman_tracks_clean_trajectory(void)
{
    float err, apogee;
    run_flight(FUSION_KALMAN, 0.0f, &err, &apogee);
    TEST_ASSERT_TRUE(err < 5.0f);
}

static void test_complementary_tracks_clean_trajectory(void)
{
    float err, apogee;
    run_flight(FUSION_COMPLEMENTARY, 0.0f, &err, &apogee);
    TEST_ASSERT_TRUE(err < 8.0f);
}

static void test_kalman_apogee_accuracy(void)
{
    rocket_sim_params_t p;
    rocket_sim_default_params(&p);
    float true_apogee = 0.0f;
    rocket_sim_true_apogee(&p, &true_apogee, NULL);

    float err, apogee;
    run_flight(FUSION_KALMAN, 1.0f, &err, &apogee);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, true_apogee, apogee);
}

static void test_kalman_robust_to_noise(void)
{
    float err, apogee;
    run_flight(FUSION_KALMAN, 1.0f, &err, &apogee);
    /* Even with sensor noise the peak error stays bounded. */
    TEST_ASSERT_TRUE(err < 15.0f);
}

static void test_first_update_initializes(void)
{
    fusion_t f;
    fusion_init(&f, FUSION_KALMAN, AEROPILOT_SEA_LEVEL_PRESSURE_PA);
    sensor_sample_t s = {0};
    s.t = 0.0f;
    s.az = AEROPILOT_GRAVITY_MS2;
    s.pressure = AEROPILOT_SEA_LEVEL_PRESSURE_PA;
    state_estimate_t est = fusion_update(&f, &s);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, est.altitude);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_kalman_tracks_clean_trajectory);
    RUN_TEST(test_complementary_tracks_clean_trajectory);
    RUN_TEST(test_kalman_apogee_accuracy);
    RUN_TEST(test_kalman_robust_to_noise);
    RUN_TEST(test_first_update_initializes);
    return UNITY_END();
}
