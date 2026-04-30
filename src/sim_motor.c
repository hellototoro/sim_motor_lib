#include "sim_motor/sim_motor.h"

static float sim_motor_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static float sim_motor_clampf(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static float sim_motor_positive_or_zero(float value)
{
    return value > 0.0f ? value : 0.0f;
}

static sim_motor_config_t sim_motor_sanitize_config(const sim_motor_config_t *config)
{
    sim_motor_config_t sanitized;

    if (config != 0) {
        sanitized = *config;
    } else {
        sanitized.max_setpoint_accel_rpm_s = 1000.0f;
        sanitized.max_setpoint_decel_rpm_s = 1000.0f;
        sanitized.max_motor_accel_rpm_s = 800.0f;
        sanitized.max_motor_decel_rpm_s = 800.0f;
        sanitized.max_abs_rpm = 3000.0f;
    }

    sanitized.max_setpoint_accel_rpm_s =
        sim_motor_positive_or_zero(sanitized.max_setpoint_accel_rpm_s);
    sanitized.max_setpoint_decel_rpm_s =
        sim_motor_positive_or_zero(sanitized.max_setpoint_decel_rpm_s);
    sanitized.max_motor_accel_rpm_s =
        sim_motor_positive_or_zero(sanitized.max_motor_accel_rpm_s);
    sanitized.max_motor_decel_rpm_s =
        sim_motor_positive_or_zero(sanitized.max_motor_decel_rpm_s);
    sanitized.max_abs_rpm = sim_motor_absf(sanitized.max_abs_rpm);

    return sanitized;
}

static int sim_motor_same_direction(float a, float b)
{
    return (a >= 0.0f && b >= 0.0f) || (a <= 0.0f && b <= 0.0f);
}

static float sim_motor_select_rate(float current,
                                   float target,
                                   float accel_rate,
                                   float decel_rate)
{
    const float current_abs = sim_motor_absf(current);
    const float target_abs = sim_motor_absf(target);

    if (current == target) {
        return 0.0f;
    }

    if (current == 0.0f) {
        return accel_rate;
    }

    if (sim_motor_same_direction(current, target) && target_abs > current_abs) {
        return accel_rate;
    }

    return decel_rate;
}

static float sim_motor_approach(float current,
                                float target,
                                float accel_rate,
                                float decel_rate,
                                float dt_s)
{
    const float error = target - current;
    const float rate = sim_motor_select_rate(current, target, accel_rate, decel_rate);
    const float max_delta = rate * dt_s;

    if (error > max_delta) {
        return current + max_delta;
    }
    if (error < -max_delta) {
        return current - max_delta;
    }
    return target;
}

void sim_motor_init(sim_motor_t *motor, const sim_motor_config_t *config)
{
    if (motor == 0) {
        return;
    }

    motor->config = sim_motor_sanitize_config(config);
    motor->requested_rpm = 0.0f;
    motor->setpoint_rpm = 0.0f;
    motor->actual_rpm = 0.0f;
    motor->enabled = 0;
    motor->faulted = 0;
}

void sim_motor_enable(sim_motor_t *motor, int enabled)
{
    if (motor == 0) {
        return;
    }

    if (!enabled) {
        motor->enabled = 0;
        return;
    }

    if (!motor->faulted) {
        motor->enabled = 1;
    }
}

void sim_motor_set_target_rpm(sim_motor_t *motor, float rpm)
{
    if (motor == 0 || motor->faulted) {
        return;
    }

    motor->requested_rpm = sim_motor_clampf(
        rpm, -motor->config.max_abs_rpm, motor->config.max_abs_rpm);
}

void sim_motor_set_fault(sim_motor_t *motor, int faulted)
{
    if (motor == 0) {
        return;
    }

    if (faulted) {
        motor->faulted = 1;
        motor->enabled = 0;
    } else {
        motor->faulted = 0;
    }
}

void sim_motor_reset_fault(sim_motor_t *motor)
{
    if (motor == 0) {
        return;
    }

    motor->faulted = 0;
}

void sim_motor_update(sim_motor_t *motor, float dt_s)
{
    float target_setpoint_rpm;

    if (motor == 0 || dt_s <= 0.0f) {
        return;
    }

    target_setpoint_rpm = (motor->enabled && !motor->faulted)
                              ? motor->requested_rpm
                              : 0.0f;

    motor->setpoint_rpm = sim_motor_approach(
        motor->setpoint_rpm,
        target_setpoint_rpm,
        motor->config.max_setpoint_accel_rpm_s,
        motor->config.max_setpoint_decel_rpm_s,
        dt_s);

    motor->actual_rpm = sim_motor_approach(
        motor->actual_rpm,
        motor->setpoint_rpm,
        motor->config.max_motor_accel_rpm_s,
        motor->config.max_motor_decel_rpm_s,
        dt_s);
}

float sim_motor_get_requested_rpm(const sim_motor_t *motor)
{
    return motor != 0 ? motor->requested_rpm : 0.0f;
}

float sim_motor_get_setpoint_rpm(const sim_motor_t *motor)
{
    return motor != 0 ? motor->setpoint_rpm : 0.0f;
}

float sim_motor_get_actual_rpm(const sim_motor_t *motor)
{
    return motor != 0 ? motor->actual_rpm : 0.0f;
}

int sim_motor_is_enabled(const sim_motor_t *motor)
{
    return motor != 0 && motor->enabled && !motor->faulted;
}

int sim_motor_is_faulted(const sim_motor_t *motor)
{
    return motor != 0 && motor->faulted;
}
