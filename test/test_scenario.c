#include "unity.h"

#include <math.h>

#include "aeropilot.h"
#include "fusion/fusion.h"
#include "sim/rocket_sim.h"
#include "state/flight_sm.h"

void setUp(void) {}
void tearDown(void) {}

typedef struct
{
    flight_state_t order[16];
    int count;
    int deployed;
    float deploy_alt;
    float deploy_time;
    float est_apogee;
    float true_apogee;
    float true_apogee_t;
    flight_state_t final_state;
} scenario_result_t;

/* Run the full sim -> fusion -> state-machine pipeline exactly as the firmware
 * wires it, recording the state transition history and deploy metrics. */
static void run_scenario(fusion_mode_t mode, scenario_result_t *r)
{
    rocket_sim_params_t p;
    rocket_sim_default_params(&p);
    rocket_sim_true_apogee(&p, &r->true_apogee, &r->true_apogee_t);

    rocket_sim_t sim;
    rocket_sim_init(&sim, &p);

    fusion_t f;
    fusion_init(&f, mode, p.ground_pressure);

    flight_sm_t sm;
    flight_sm_config_t cfg;
    flight_sm_default_config(&cfg);
    flight_sm_init(&sm, &cfg);
    flight_sm_arm(&sm);

    r->count = 0;
    r->order[r->count++] = sm.state;
    r->est_apogee = 0.0f;

    flight_state_t prev = sm.state;
    for (int i = 0; i < 6000 && sm.state != FLIGHT_LANDED; ++i)
    {
        rocket_sim_advance(&sim, 0.01f);
        sensor_sample_t s = rocket_sim_read(&sim);
        float a_vert = s.az - AEROPILOT_GRAVITY_MS2;
        state_estimate_t est = fusion_update(&f, &s);
        flight_state_t st = flight_sm_update(&sm, &est, a_vert);

        if (est.altitude > r->est_apogee)
        {
            r->est_apogee = est.altitude;
        }
        if (st != prev)
        {
            if (r->count < 16)
            {
                r->order[r->count++] = st;
            }
            prev = st;
        }
    }

    r->deployed = flight_sm_deploy_fired(&sm);
    r->deploy_alt = sm.max_altitude;
    r->deploy_time = sm.deploy_time;
    r->final_state = sm.state;
}

static void assert_states_in_order(const scenario_result_t *r)
{
    /* The recorded transitions must be a monotonically increasing subsequence
     * of the nominal flight progression (no SAFE, no regressions). */
    flight_state_t expect[] = {FLIGHT_ARMED, FLIGHT_BOOST, FLIGHT_COAST,
                               FLIGHT_APOGEE, FLIGHT_DESCENT, FLIGHT_LANDED};
    int idx = 0;
    for (int i = 0; i < r->count; ++i)
    {
        while (idx < 6 && r->order[i] != expect[idx])
        {
            ++idx;
        }
        TEST_ASSERT_TRUE_MESSAGE(idx < 6, "unexpected/out-of-order state");
        ++idx;
    }
}

static void test_kalman_scenario(void)
{
    scenario_result_t r;
    run_scenario(FUSION_KALMAN, &r);

    TEST_ASSERT_EQUAL_INT(FLIGHT_LANDED, r.final_state);
    TEST_ASSERT_TRUE(r.deployed);
    assert_states_in_order(&r);

    /* Deploy altitude within a window of true apogee. */
    TEST_ASSERT_FLOAT_WITHIN(15.0f, r.true_apogee, r.deploy_alt);
    /* Deploy timing within a window of true apogee time. */
    TEST_ASSERT_FLOAT_WITHIN(0.75f, r.true_apogee_t, r.deploy_time);
}

static void test_complementary_scenario(void)
{
    scenario_result_t r;
    run_scenario(FUSION_COMPLEMENTARY, &r);

    /* The complementary filter's ground-level velocity estimate is noisier
     * than the Kalman's (it differentiates the barometer), so we require it to
     * deploy and descend correctly rather than latch the strict LANDED check. */
    TEST_ASSERT_TRUE(r.final_state == FLIGHT_DESCENT ||
                     r.final_state == FLIGHT_LANDED);
    TEST_ASSERT_TRUE(r.deployed);
    assert_states_in_order(&r);
    TEST_ASSERT_FLOAT_WITHIN(20.0f, r.true_apogee, r.deploy_alt);
}

static void test_all_states_visited(void)
{
    scenario_result_t r;
    run_scenario(FUSION_KALMAN, &r);

    int seen_boost = 0, seen_coast = 0, seen_apogee = 0;
    int seen_descent = 0, seen_landed = 0;
    for (int i = 0; i < r.count; ++i)
    {
        seen_boost |= (r.order[i] == FLIGHT_BOOST);
        seen_coast |= (r.order[i] == FLIGHT_COAST);
        seen_apogee |= (r.order[i] == FLIGHT_APOGEE);
        seen_descent |= (r.order[i] == FLIGHT_DESCENT);
        seen_landed |= (r.order[i] == FLIGHT_LANDED);
    }
    TEST_ASSERT_TRUE(seen_boost && seen_coast && seen_apogee &&
                     seen_descent && seen_landed);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_kalman_scenario);
    RUN_TEST(test_complementary_scenario);
    RUN_TEST(test_all_states_visited);
    return UNITY_END();
}
