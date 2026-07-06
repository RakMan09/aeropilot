#include "unity.h"

#include "watchdog/watchdog.h"

void setUp(void) {}
void tearDown(void) {}

static void test_first_check_primes(void)
{
    watchdog_t wd;
    watchdog_init(&wd, 3, 3);
    uint32_t c[3] = {5, 10, 15};
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c));
}

static void test_all_healthy(void)
{
    watchdog_t wd;
    watchdog_init(&wd, 3, 3);
    uint32_t c[3] = {0, 0, 0};
    watchdog_check(&wd, c);
    for (int i = 1; i < 10; ++i)
    {
        c[0] = i; c[1] = i * 2; c[2] = i * 3;
        TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c));
    }
}

static void test_trips_after_max_missed(void)
{
    watchdog_t wd;
    watchdog_init(&wd, 3, 3);
    uint32_t c[3] = {0, 0, 0};
    watchdog_check(&wd, c); /* prime */

    /* Task 1 stalls; tasks 0 and 2 keep advancing. */
    c[0] = 1; c[2] = 1;
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c)); /* miss 1 */
    c[0] = 2; c[2] = 2;
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c)); /* miss 2 */
    c[0] = 3; c[2] = 3;
    TEST_ASSERT_EQUAL_INT(1, watchdog_check(&wd, c));  /* miss 3 -> trip */
}

static void test_recovers_before_trip(void)
{
    watchdog_t wd;
    watchdog_init(&wd, 2, 3);
    uint32_t c[2] = {0, 0};
    watchdog_check(&wd, c);

    c[0] = 0; c[1] = 1; /* task0 misses once */
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c));
    c[0] = 1; c[1] = 2; /* task0 recovers */
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c));
    /* Now it would take another 3 consecutive misses to trip. */
    c[1] = 3;
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c)); /* miss 1 */
    c[1] = 4;
    TEST_ASSERT_EQUAL_INT(-1, watchdog_check(&wd, c)); /* miss 2 */
    c[1] = 5;
    TEST_ASSERT_EQUAL_INT(0, watchdog_check(&wd, c));  /* trip on task0 */
}

static void test_reports_lowest_index_first(void)
{
    watchdog_t wd;
    watchdog_init(&wd, 3, 1); /* trip on first miss */
    uint32_t c[3] = {0, 0, 0};
    watchdog_check(&wd, c);
    /* Tasks 1 and 2 both stall; index 1 should be reported. */
    c[0] = 1;
    TEST_ASSERT_EQUAL_INT(1, watchdog_check(&wd, c));
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_first_check_primes);
    RUN_TEST(test_all_healthy);
    RUN_TEST(test_trips_after_max_missed);
    RUN_TEST(test_recovers_before_trip);
    RUN_TEST(test_reports_lowest_index_first);
    return UNITY_END();
}
