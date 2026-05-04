#ifndef SIM_MOTOR_SIM_MOTOR_H
#define SIM_MOTOR_SIM_MOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float max_setpoint_accel_rpm_s;
    float max_setpoint_decel_rpm_s;
    float max_motor_accel_rpm_s;
    float max_motor_decel_rpm_s;
    float max_abs_rpm;
    float steady_noise_ratio;
} sim_motor_config_t;

typedef struct sim_motor_t {
    sim_motor_config_t config;
    float requested_rpm;
    float setpoint_rpm;
    float actual_rpm;
    float position_rev;
    uint32_t noise_seed;
    int enabled;
    int faulted;
} sim_motor_t;

void sim_motor_init(sim_motor_t *motor, const sim_motor_config_t *config);
void sim_motor_enable(sim_motor_t *motor, int enabled);
void sim_motor_set_target_rpm(sim_motor_t *motor, float rpm);
void sim_motor_set_fault(sim_motor_t *motor, int faulted);
void sim_motor_reset_fault(sim_motor_t *motor);

void sim_motor_update(sim_motor_t *motor, float dt_s);

float sim_motor_get_requested_rpm(const sim_motor_t *motor);
float sim_motor_get_setpoint_rpm(const sim_motor_t *motor);
float sim_motor_get_actual_rpm(const sim_motor_t *motor);
float sim_motor_get_position_rev(const sim_motor_t *motor);
void sim_motor_reset_position(sim_motor_t *motor);
void sim_motor_set_noise_seed(sim_motor_t *motor, uint32_t seed);
int sim_motor_is_enabled(const sim_motor_t *motor);
int sim_motor_is_faulted(const sim_motor_t *motor);

#ifdef __cplusplus
}
#endif

#endif
