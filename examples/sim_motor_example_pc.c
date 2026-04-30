#include "sim_motor/sim_motor.h"

#include <stdio.h>

static float command_for_time(float time_s)
{
    if (time_s < 1.0f) {
        return 0.0f;
    }
    if (time_s < 4.0f) {
        return 1000.0f;
    }
    if (time_s < 7.0f) {
        return -500.0f;
    }
    return 0.0f;
}

int main(void)
{
    const sim_motor_config_t config = {
        1500.0f,
        2000.0f,
        900.0f,
        1200.0f,
        3000.0f,
    };
    const float dt_s = 0.01f;
    const float duration_s = 9.0f;
    sim_motor_t motor;
    float time_s;

    sim_motor_init(&motor, &config);
    sim_motor_enable(&motor, 1);

    printf("time,requested,setpoint,actual,enabled,fault\n");

    for (time_s = 0.0f; time_s <= duration_s + 0.0001f; time_s += dt_s) {
        sim_motor_set_target_rpm(&motor, command_for_time(time_s));
        sim_motor_update(&motor, dt_s);

        printf("%.2f,%.3f,%.3f,%.3f,%d,%d\n",
               time_s,
               sim_motor_get_requested_rpm(&motor),
               sim_motor_get_setpoint_rpm(&motor),
               sim_motor_get_actual_rpm(&motor),
               sim_motor_is_enabled(&motor),
               sim_motor_is_faulted(&motor));
    }

    return 0;
}
