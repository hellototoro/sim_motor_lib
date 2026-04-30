#include "sim_motor/sim_motor_can.h"

#include <stdint.h>
#include <stdio.h>

#define TEST_EPSILON 0.0001f

static int g_failures = 0;

static void expect_true(int condition, const char *message)
{
    if (!condition) {
        ++g_failures;
        printf("FAIL: %s\n", message);
    }
}

static void expect_int(int actual, int expected, const char *message)
{
    if (actual != expected) {
        ++g_failures;
        printf("FAIL: %s (actual %d expected %d)\n", message, actual, expected);
    }
}

static float test_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static void expect_near(float actual, float expected, float epsilon, const char *message)
{
    if (test_absf(actual - expected) > epsilon) {
        ++g_failures;
        printf("FAIL: %s (actual %.6f expected %.6f)\n", message, actual, expected);
    }
}

static void write_i32_le(uint8_t *data, int32_t value)
{
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)((raw >> 8) & 0xFFu);
    data[2] = (uint8_t)((raw >> 16) & 0xFFu);
    data[3] = (uint8_t)((raw >> 24) & 0xFFu);
}

static int32_t read_i32_le(const uint8_t *data)
{
    uint32_t raw = (uint32_t)data[0] |
                   ((uint32_t)data[1] << 8) |
                   ((uint32_t)data[2] << 16) |
                   ((uint32_t)data[3] << 24);
    return (int32_t)raw;
}

static sim_motor_can_node_config_t test_config(void)
{
    sim_motor_can_node_config_t config;

    config.node_id = 5u;
    config.command_timeout_ms = 100u;
    config.status_period_ms = 20u;
    config.speed_period_ms = 30u;
    config.motor_config.max_setpoint_accel_rpm_s = 100.0f;
    config.motor_config.max_setpoint_decel_rpm_s = 200.0f;
    config.motor_config.max_motor_accel_rpm_s = 50.0f;
    config.motor_config.max_motor_decel_rpm_s = 80.0f;
    config.motor_config.max_abs_rpm = 500.0f;

    return config;
}

static sim_motor_can_frame_t control_frame(uint8_t node_id, uint8_t command, uint8_t seq)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_CONTROL_BASE + node_id);
    frame.dlc = 2u;
    frame.data[0] = command;
    frame.data[1] = seq;
    frame.data[2] = 0u;
    frame.data[3] = 0u;
    frame.data[4] = 0u;
    frame.data[5] = 0u;
    frame.data[6] = 0u;
    frame.data[7] = 0u;

    return frame;
}

static sim_motor_can_frame_t speed_frame(uint8_t node_id, int32_t rpm_x100, uint8_t seq)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_SPEED_COMMAND_BASE + node_id);
    frame.dlc = 6u;
    write_i32_le(&frame.data[0], rpm_x100);
    frame.data[4] = seq;
    frame.data[5] = 0u;
    frame.data[6] = 0u;
    frame.data[7] = 0u;

    return frame;
}

static sim_motor_can_frame_t query_frame(uint8_t node_id, uint8_t query, uint8_t seq)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_QUERY_BASE + node_id);
    frame.dlc = 2u;
    frame.data[0] = query;
    frame.data[1] = seq;
    frame.data[2] = 0u;
    frame.data[3] = 0u;
    frame.data[4] = 0u;
    frame.data[5] = 0u;
    frame.data[6] = 0u;
    frame.data[7] = 0u;

    return frame;
}

static int drain_one(sim_motor_can_node_t *node, sim_motor_can_frame_t *frame)
{
    return sim_motor_can_node_next_tx(node, frame);
}

static void test_id_matching_and_broadcast(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t frame;
    sim_motor_can_frame_t tx;
    sim_motor_can_node_config_t config = test_config();

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    frame = control_frame(6u, SIM_MOTOR_CAN_CMD_ENABLE, 1u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE,
               "frame for a different node is ignored");
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_ERROR_TX_EMPTY,
               "ignored frame does not enqueue a response");

    frame = control_frame(0u, SIM_MOTOR_CAN_CMD_ENABLE, 2u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "broadcast control applies to node");
    expect_int(sim_motor_can_node_get_state(&node), SIM_MOTOR_CAN_STATE_READY,
               "broadcast enable moves node to ready");
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_ERROR_TX_EMPTY,
               "broadcast command does not enqueue an ack");
}

static void test_speed_command_encoding_and_ramp(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t frame;
    sim_motor_can_frame_t tx;
    const sim_motor_t *motor;
    sim_motor_can_node_config_t config = test_config();

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_ENABLE, 10u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "enable command succeeds");
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_OK, "enable ack is queued");
    expect_int(tx.id, SIM_MOTOR_CAN_ID_ACK_BASE + 5u, "enable ack id matches node");

    frame = speed_frame(5u, -12345, 11u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "speed command succeeds");
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_OK, "speed ack is queued");
    expect_int(read_i32_le(&tx.data[4]), -12345, "speed ack detail uses rpm_x100");

    motor = sim_motor_can_node_get_motor(&node);
    expect_near(sim_motor_get_requested_rpm(motor), -123.45f, TEST_EPSILON,
                "rpm_x100 command converts to rpm");

    sim_motor_can_node_update(&node, 0.1f, 10u);
    expect_near(sim_motor_get_setpoint_rpm(motor), -10.0f, TEST_EPSILON,
                "setpoint ramp follows acceleration limit");
    expect_near(sim_motor_get_actual_rpm(motor), -5.0f, TEST_EPSILON,
                "actual speed ramp follows motor acceleration limit");
}

static void test_control_state_machine_and_fault_reset(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t frame;
    sim_motor_can_frame_t tx;
    sim_motor_can_node_config_t config = test_config();

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_ENABLE, 20u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "enable command succeeds");
    (void)drain_one(&node, &tx);
    expect_int(sim_motor_can_node_get_state(&node), SIM_MOTOR_CAN_STATE_READY,
               "enable moves to ready");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_INJECT_FAULT, 21u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "inject fault command succeeds");
    (void)drain_one(&node, &tx);
    expect_int(sim_motor_can_node_get_state(&node), SIM_MOTOR_CAN_STATE_FAULT,
               "manual fault moves to fault");
    expect_int(sim_motor_can_node_get_fault_code(&node), SIM_MOTOR_CAN_FAULT_MANUAL,
               "manual fault code is stored");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_ENABLE, 22u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_ERROR_BAD_FRAME,
               "enable is rejected while faulted");
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_OK, "bad-state ack is queued");
    expect_int(tx.data[1], SIM_MOTOR_CAN_ACK_BAD_STATE, "ack reports bad state");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_FAULT_RESET, 23u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "fault reset succeeds");
    (void)drain_one(&node, &tx);
    expect_int(sim_motor_can_node_get_state(&node), SIM_MOTOR_CAN_STATE_DISABLED,
               "fault reset moves to disabled");
    expect_int(sim_motor_can_node_get_fault_code(&node), SIM_MOTOR_CAN_FAULT_NONE,
               "fault reset clears fault code");
}

static void test_timeout_enters_fault_and_stops(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t frame;
    sim_motor_can_frame_t tx;
    const sim_motor_t *motor;
    sim_motor_can_node_config_t config = test_config();

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    frame = control_frame(5u, SIM_MOTOR_CAN_CMD_ENABLE, 30u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "enable starts watchdog");
    (void)drain_one(&node, &tx);

    frame = speed_frame(5u, 20000, 31u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
               "speed command starts motion");
    (void)drain_one(&node, &tx);

    sim_motor_can_node_update(&node, 0.1f, 50u);
    sim_motor_can_node_update(&node, 0.1f, 60u);

    expect_int(sim_motor_can_node_get_state(&node), SIM_MOTOR_CAN_STATE_FAULT,
               "command timeout enters fault");
    expect_int(sim_motor_can_node_get_fault_code(&node), SIM_MOTOR_CAN_FAULT_COMM_TIMEOUT,
               "timeout fault code is stored");

    motor = sim_motor_can_node_get_motor(&node);
    expect_true(!sim_motor_is_enabled(motor), "timeout disables motor output");
}

static void test_periodic_status_and_speed_frames(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t tx;
    sim_motor_can_node_config_t config = test_config();

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    sim_motor_can_node_update(&node, 0.02f, 20u);
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_OK, "periodic status is queued");
    expect_int(tx.id, SIM_MOTOR_CAN_ID_STATUS_BASE + 5u, "status id matches node");
    expect_int(tx.dlc, 8u, "status dlc is 8");

    sim_motor_can_node_update(&node, 0.01f, 10u);
    expect_int(drain_one(&node, &tx), SIM_MOTOR_CAN_OK, "periodic speed is queued");
    expect_int(tx.id, SIM_MOTOR_CAN_ID_SPEED_FEEDBACK_BASE + 5u, "speed id matches node");
    expect_int(tx.dlc, 8u, "speed dlc is 8");
}

static void test_tx_queue_full_returns_error(void)
{
    sim_motor_can_node_t node;
    sim_motor_can_frame_t frame;
    sim_motor_can_node_config_t config = test_config();
    unsigned int i;

    expect_int(sim_motor_can_node_init(&node, &config), SIM_MOTOR_CAN_OK,
               "node init succeeds");

    for (i = 0u; i < SIM_MOTOR_CAN_TX_QUEUE_SIZE; ++i) {
        frame = query_frame(5u, SIM_MOTOR_CAN_QUERY_STATUS, (uint8_t)i);
        expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_OK,
                   "query fills tx queue");
    }

    frame = query_frame(5u, SIM_MOTOR_CAN_QUERY_STATUS, 99u);
    expect_int(sim_motor_can_node_receive(&node, &frame), SIM_MOTOR_CAN_ERROR_TX_FULL,
               "full tx queue returns explicit error");
}

int main(void)
{
    test_id_matching_and_broadcast();
    test_speed_command_encoding_and_ramp();
    test_control_state_machine_and_fault_reset();
    test_timeout_enters_fault_and_stops();
    test_periodic_status_and_speed_frames();
    test_tx_queue_full_returns_error();

    if (g_failures != 0) {
        printf("%d test failure(s)\n", g_failures);
        return 1;
    }

    printf("All sim_motor_can tests passed\n");
    return 0;
}
