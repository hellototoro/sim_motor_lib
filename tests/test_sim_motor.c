#include "sim_motor/sim_motor.h"

#include <stdio.h>

#define TEST_EPSILON 0.0001f

static int g_failures = 0;

static float test_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static void expect_near(float actual, float expected, float epsilon, const char *message)
{
    if (test_absf(actual - expected) > epsilon) {
        ++g_failures;
        printf("FAIL: %s (actual %.6f expected %.6f)\n", message, actual, expected);
    }
}

static sim_motor_config_t test_config(void)
{
    sim_motor_config_t config;
    config.max_setpoint_accel_rpm_s = 100.0f;
    config.max_setpoint_decel_rpm_s = 200.0f;
    config.max_motor_accel_rpm_s = 50.0f;
    config.max_motor_decel_rpm_s = 80.0f;
    config.max_abs_rpm = 500.0f;
    config.steady_noise_ratio = -1.0f;
    return config;
}

static void test_accel_limits_setpoint_and_actual(void)
{
    const sim_motor_config_t config = test_config();
    const float dt_s = 0.1f;
    sim_motor_t motor;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 300.0f);
    sim_motor_update(&motor, dt_s);

    expect_near(sim_motor_get_setpoint_rpm(&motor), 10.0f, TEST_EPSILON,
                "setpoint is limited by setpoint acceleration");
    expect_near(sim_motor_get_actual_rpm(&motor), 5.0f, TEST_EPSILON,
                "actual speed is limited by motor acceleration");
}

static void test_target_is_clamped(void)
{
    const sim_motor_config_t config = test_config();
    sim_motor_t motor;

    sim_motor_init(&motor, &config);
    sim_motor_set_target_rpm(&motor, 900.0f);

    expect_near(sim_motor_get_requested_rpm(&motor), 500.0f, TEST_EPSILON,
                "positive target is clamped to max rpm");

    sim_motor_set_target_rpm(&motor, -900.0f);

    expect_near(sim_motor_get_requested_rpm(&motor), -500.0f, TEST_EPSILON,
                "negative target is clamped to min rpm");
}

static void test_reverse_direction_is_continuous(void)
{
    const sim_motor_config_t config = test_config();
    const float dt_s = 0.1f;
    sim_motor_t motor;
    float previous_setpoint;
    float previous_actual;
    int i;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 200.0f);

    for (i = 0; i < 30; ++i) {
        sim_motor_update(&motor, dt_s);
    }

    sim_motor_set_target_rpm(&motor, -200.0f);

    for (i = 0; i < 40; ++i) {
        previous_setpoint = sim_motor_get_setpoint_rpm(&motor);
        previous_actual = sim_motor_get_actual_rpm(&motor);
        sim_motor_update(&motor, dt_s);

        expect_true(test_absf(sim_motor_get_setpoint_rpm(&motor) - previous_setpoint)
                        <= config.max_setpoint_decel_rpm_s * dt_s + TEST_EPSILON,
                    "reverse setpoint step is deceleration-limited");
        expect_true(test_absf(sim_motor_get_actual_rpm(&motor) - previous_actual)
                        <= config.max_motor_decel_rpm_s * dt_s + TEST_EPSILON,
                    "reverse actual step is deceleration-limited");
    }

    expect_true(sim_motor_get_setpoint_rpm(&motor) < 0.0f,
                "setpoint eventually crosses through zero");
}

static void test_disable_ramps_to_zero(void)
{
    const sim_motor_config_t config = test_config();
    const float dt_s = 0.1f;
    sim_motor_t motor;
    int i;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 200.0f);

    for (i = 0; i < 30; ++i) {
        sim_motor_update(&motor, dt_s);
    }

    sim_motor_enable(&motor, 0);
    sim_motor_update(&motor, dt_s);

    expect_true(!sim_motor_is_enabled(&motor), "disabled motor reports not enabled");
    expect_near(sim_motor_get_setpoint_rpm(&motor), 180.0f, TEST_EPSILON,
                "disabled motor decelerates setpoint toward zero");

    for (i = 0; i < 40; ++i) {
        sim_motor_update(&motor, dt_s);
    }

    expect_near(sim_motor_get_setpoint_rpm(&motor), 0.0f, TEST_EPSILON,
                "disabled motor setpoint reaches zero");
    expect_near(sim_motor_get_actual_rpm(&motor), 0.0f, TEST_EPSILON,
                "disabled motor actual speed reaches zero");
}

static void test_fault_ignores_new_target_until_reset(void)
{
    const sim_motor_config_t config = test_config();
    const float dt_s = 0.1f;
    sim_motor_t motor;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 200.0f);
    sim_motor_update(&motor, dt_s);

    sim_motor_set_fault(&motor, 1);
    sim_motor_set_target_rpm(&motor, 400.0f);

    expect_true(sim_motor_is_faulted(&motor), "faulted motor reports fault");
    expect_true(!sim_motor_is_enabled(&motor), "faulted motor reports not enabled");
    expect_near(sim_motor_get_requested_rpm(&motor), 200.0f, TEST_EPSILON,
                "faulted motor ignores new target");

    sim_motor_update(&motor, dt_s);
    expect_near(sim_motor_get_setpoint_rpm(&motor), 0.0f, TEST_EPSILON,
                "faulted motor ramps setpoint toward zero");

    sim_motor_reset_fault(&motor);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 400.0f);

    expect_true(!sim_motor_is_faulted(&motor), "reset clears fault");
    expect_true(sim_motor_is_enabled(&motor), "motor can be re-enabled after reset");
    expect_near(sim_motor_get_requested_rpm(&motor), 400.0f, TEST_EPSILON,
                "target can be updated after reset");
}

static void test_non_positive_dt_does_not_change_state(void)
{
    const sim_motor_config_t config = test_config();
    sim_motor_t motor;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 300.0f);

    sim_motor_update(&motor, 0.0f);
    expect_near(sim_motor_get_setpoint_rpm(&motor), 0.0f, TEST_EPSILON,
                "zero dt leaves setpoint unchanged");
    expect_near(sim_motor_get_actual_rpm(&motor), 0.0f, TEST_EPSILON,
                "zero dt leaves actual unchanged");
    expect_near(sim_motor_get_position_rev(&motor), 0.0f, TEST_EPSILON,
                "zero dt leaves position unchanged");

    sim_motor_update(&motor, -0.1f);
    expect_near(sim_motor_get_setpoint_rpm(&motor), 0.0f, TEST_EPSILON,
                "negative dt leaves setpoint unchanged");
    expect_near(sim_motor_get_actual_rpm(&motor), 0.0f, TEST_EPSILON,
                "negative dt leaves actual unchanged");
    expect_near(sim_motor_get_position_rev(&motor), 0.0f, TEST_EPSILON,
                "negative dt leaves position unchanged");
}

static void test_position_integrates_signed_revolutions(void)
{
    sim_motor_config_t config = test_config();
    sim_motor_t motor;

    config.max_setpoint_accel_rpm_s = 1000.0f;
    config.max_motor_accel_rpm_s = 1000.0f;
    config.max_setpoint_decel_rpm_s = 1000.0f;
    config.max_motor_decel_rpm_s = 1000.0f;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);
    sim_motor_set_target_rpm(&motor, 60.0f);
    sim_motor_update(&motor, 1.0f);

    expect_near(sim_motor_get_position_rev(&motor), 1.0f, TEST_EPSILON,
                "positive rpm increases signed position in revolutions");

    sim_motor_set_target_rpm(&motor, -60.0f);
    sim_motor_update(&motor, 1.0f);

    expect_near(sim_motor_get_position_rev(&motor), 0.0f, TEST_EPSILON,
                "negative rpm decreases signed position in revolutions");

    sim_motor_reset_position(&motor);
    expect_near(sim_motor_get_position_rev(&motor), 0.0f, TEST_EPSILON,
                "position reset clears signed position");
}

static void test_steady_speed_has_seeded_noise(void)
{
    sim_motor_config_t config = test_config();
    sim_motor_t motor_a;
    sim_motor_t motor_b;
    float actual_a;
    float actual_b;

    config.max_setpoint_accel_rpm_s = 1000.0f;
    config.max_motor_accel_rpm_s = 1000.0f;
    config.steady_noise_ratio = 0.0f;

    sim_motor_init(&motor_a, &config);
    sim_motor_init(&motor_b, &config);
    sim_motor_set_noise_seed(&motor_a, 7u);
    sim_motor_set_noise_seed(&motor_b, 7u);
    sim_motor_enable(&motor_a, 1);
    sim_motor_enable(&motor_b, 1);
    sim_motor_set_target_rpm(&motor_a, 100.0f);
    sim_motor_set_target_rpm(&motor_b, 100.0f);

    sim_motor_update(&motor_a, 1.0f);
    sim_motor_update(&motor_b, 1.0f);

    actual_a = sim_motor_get_actual_rpm(&motor_a);
    actual_b = sim_motor_get_actual_rpm(&motor_b);

    expect_true(test_absf(actual_a - sim_motor_get_setpoint_rpm(&motor_a)) <= 1.0f,
                "steady noise stays within one percent of setpoint");
    expect_true(test_absf(actual_a - sim_motor_get_setpoint_rpm(&motor_a)) > TEST_EPSILON,
                "steady noise keeps actual speed from staying fixed at setpoint");
    expect_near(actual_a, actual_b, TEST_EPSILON,
                "seeded steady noise is reproducible");
}

int main(void)
{
    test_accel_limits_setpoint_and_actual();
    test_target_is_clamped();
    test_reverse_direction_is_continuous();
    test_disable_ramps_to_zero();
    test_fault_ignores_new_target_until_reset();
    test_non_positive_dt_does_not_change_state();
    test_position_integrates_signed_revolutions();
    test_steady_speed_has_seeded_noise();

    if (g_failures != 0) {
        printf("%d test failure(s)\n", g_failures);
        return 1;
    }

    printf("All sim_motor tests passed\n");
    return 0;
}
