#include "sim_motor/sim_motor_can.h"

#define SIM_MOTOR_CAN_FLAG_ENABLED 0x01u
#define SIM_MOTOR_CAN_FLAG_FAULTED 0x02u
#define SIM_MOTOR_CAN_FLAG_MOVING 0x04u
#define SIM_MOTOR_CAN_FLAG_WATCHDOG_ACTIVE 0x08u

#define SIM_MOTOR_CAN_STOP_EPSILON_RPM 0.001f
#define SIM_MOTOR_CAN_PROTOCOL_VERSION 2u

static int32_t can_read_i32_le(const uint8_t *data)
{
    uint32_t value = (uint32_t)data[0] |
                     ((uint32_t)data[1] << 8) |
                     ((uint32_t)data[2] << 16) |
                     ((uint32_t)data[3] << 24);
    return (int32_t)value;
}

static void can_write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static void can_write_u32_le(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
    data[2] = (uint8_t)((value >> 16) & 0xFFu);
    data[3] = (uint8_t)((value >> 24) & 0xFFu);
}

static void can_write_i32_le(uint8_t *data, int32_t value)
{
    can_write_u32_le(data, (uint32_t)value);
}

static void can_write_i64_le(uint8_t *data, int64_t value)
{
    uint64_t raw = (uint64_t)value;
    data[0] = (uint8_t)(raw & 0xFFu);
    data[1] = (uint8_t)((raw >> 8) & 0xFFu);
    data[2] = (uint8_t)((raw >> 16) & 0xFFu);
    data[3] = (uint8_t)((raw >> 24) & 0xFFu);
    data[4] = (uint8_t)((raw >> 32) & 0xFFu);
    data[5] = (uint8_t)((raw >> 40) & 0xFFu);
    data[6] = (uint8_t)((raw >> 48) & 0xFFu);
    data[7] = (uint8_t)((raw >> 56) & 0xFFu);
}

static float can_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static int32_t rpm_to_x100(float rpm)
{
    float scaled = rpm * 100.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int32_t)scaled;
}

static float rpm_from_x100(int32_t rpm_x100)
{
    return (float)rpm_x100 / 100.0f;
}

static int64_t position_to_x1000(float position_rev)
{
    float scaled = position_rev * 1000.0f;

    if (scaled >= 0.0f) {
        scaled += 0.5f;
    } else {
        scaled -= 0.5f;
    }

    return (int64_t)scaled;
}

static int tx_push(sim_motor_can_node_t *node, const sim_motor_can_frame_t *frame)
{
    if (node->tx_count >= SIM_MOTOR_CAN_TX_QUEUE_SIZE) {
        return SIM_MOTOR_CAN_ERROR_TX_FULL;
    }

    node->tx_queue[node->tx_tail] = *frame;
    node->tx_tail = (uint8_t)((node->tx_tail + 1u) % SIM_MOTOR_CAN_TX_QUEUE_SIZE);
    node->tx_count++;
    return SIM_MOTOR_CAN_OK;
}

static uint8_t status_flags(const sim_motor_can_node_t *node)
{
    uint8_t flags = 0u;

    if (sim_motor_is_enabled(&node->motor)) {
        flags |= SIM_MOTOR_CAN_FLAG_ENABLED;
    }
    if (sim_motor_is_faulted(&node->motor) || node->state == SIM_MOTOR_CAN_STATE_FAULT) {
        flags |= SIM_MOTOR_CAN_FLAG_FAULTED;
    }
    if (can_absf(sim_motor_get_actual_rpm(&node->motor)) > SIM_MOTOR_CAN_STOP_EPSILON_RPM ||
        can_absf(sim_motor_get_setpoint_rpm(&node->motor)) > SIM_MOTOR_CAN_STOP_EPSILON_RPM) {
        flags |= SIM_MOTOR_CAN_FLAG_MOVING;
    }
    if (node->watchdog_active) {
        flags |= SIM_MOTOR_CAN_FLAG_WATCHDOG_ACTIVE;
    }

    return flags;
}

static int enqueue_status(sim_motor_can_node_t *node)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_STATUS_BASE + node->config.node_id);
    frame.dlc = 8u;
    frame.data[0] = (uint8_t)node->state;
    frame.data[1] = status_flags(node);
    can_write_u16_le(&frame.data[2], node->fault_code);
    can_write_u32_le(&frame.data[4], node->uptime_ms);

    return tx_push(node, &frame);
}

static int enqueue_speed(sim_motor_can_node_t *node)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_SPEED_FEEDBACK_BASE + node->config.node_id);
    frame.dlc = 8u;
    can_write_i32_le(&frame.data[0], rpm_to_x100(sim_motor_get_actual_rpm(&node->motor)));
    can_write_i32_le(&frame.data[4], rpm_to_x100(sim_motor_get_setpoint_rpm(&node->motor)));

    return tx_push(node, &frame);
}

static int enqueue_position(sim_motor_can_node_t *node)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_POSITION_FEEDBACK_BASE + node->config.node_id);
    frame.dlc = 8u;
    can_write_i64_le(&frame.data[0],
                     position_to_x1000(sim_motor_get_position_rev(&node->motor)));

    return tx_push(node, &frame);
}

static int enqueue_ack(sim_motor_can_node_t *node,
                       uint8_t source_cmd,
                       uint8_t result,
                       uint8_t seq,
                       int32_t detail)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_ACK_BASE + node->config.node_id);
    frame.dlc = 8u;
    frame.data[0] = source_cmd;
    frame.data[1] = result;
    frame.data[2] = (uint8_t)node->state;
    frame.data[3] = seq;
    can_write_i32_le(&frame.data[4], detail);

    return tx_push(node, &frame);
}

static int enqueue_protocol_info(sim_motor_can_node_t *node)
{
    sim_motor_can_frame_t frame;

    frame.id = (uint16_t)(SIM_MOTOR_CAN_ID_ACK_BASE + node->config.node_id);
    frame.dlc = 8u;
    frame.data[0] = SIM_MOTOR_CAN_QUERY_PROTOCOL_INFO;
    frame.data[1] = SIM_MOTOR_CAN_ACK_OK;
    frame.data[2] = (uint8_t)node->state;
    frame.data[3] = SIM_MOTOR_CAN_PROTOCOL_VERSION;
    can_write_u16_le(&frame.data[4], SIM_MOTOR_CAN_MAX_NODE_ID);
    can_write_u16_le(&frame.data[6], SIM_MOTOR_CAN_TX_QUEUE_SIZE);

    return tx_push(node, &frame);
}

static void refresh_watchdog(sim_motor_can_node_t *node)
{
    if (node->config.command_timeout_ms > 0u) {
        node->watchdog_active = 1u;
        node->command_age_ms = 0u;
    }
}

static void enter_fault(sim_motor_can_node_t *node, uint16_t fault_code)
{
    node->fault_code = fault_code;
    node->state = SIM_MOTOR_CAN_STATE_FAULT;
    node->stop_target_state = SIM_MOTOR_CAN_STATE_FAULT;
    sim_motor_set_fault(&node->motor, 1);
}

static void enter_stopping(sim_motor_can_node_t *node,
                           sim_motor_can_state_t target_state)
{
    node->stop_target_state = target_state;
    node->state = SIM_MOTOR_CAN_STATE_STOPPING;
    sim_motor_set_target_rpm(&node->motor, 0.0f);
    if (target_state == SIM_MOTOR_CAN_STATE_DISABLED) {
        sim_motor_enable(&node->motor, 0);
    }
}

static int is_stopped(const sim_motor_can_node_t *node)
{
    return can_absf(sim_motor_get_actual_rpm(&node->motor)) <= SIM_MOTOR_CAN_STOP_EPSILON_RPM &&
           can_absf(sim_motor_get_setpoint_rpm(&node->motor)) <= SIM_MOTOR_CAN_STOP_EPSILON_RPM;
}

static void update_state_from_motion(sim_motor_can_node_t *node)
{
    if (node->state == SIM_MOTOR_CAN_STATE_INIT) {
        node->state = SIM_MOTOR_CAN_STATE_DISABLED;
    }

    if (node->state == SIM_MOTOR_CAN_STATE_FAULT) {
        return;
    }

    if (node->state == SIM_MOTOR_CAN_STATE_STOPPING) {
        if (is_stopped(node)) {
            node->state = node->stop_target_state;
            if (node->state == SIM_MOTOR_CAN_STATE_READY) {
                sim_motor_enable(&node->motor, 1);
            }
        }
        return;
    }

    if (node->state == SIM_MOTOR_CAN_STATE_READY &&
        can_absf(sim_motor_get_requested_rpm(&node->motor)) > SIM_MOTOR_CAN_STOP_EPSILON_RPM) {
        node->state = SIM_MOTOR_CAN_STATE_RUNNING;
    }

    if (node->state == SIM_MOTOR_CAN_STATE_RUNNING &&
        can_absf(sim_motor_get_requested_rpm(&node->motor)) <= SIM_MOTOR_CAN_STOP_EPSILON_RPM &&
        is_stopped(node)) {
        node->state = SIM_MOTOR_CAN_STATE_READY;
    }
}

static int parse_frame_node(uint16_t id, uint16_t base, uint8_t *frame_node_id)
{
    uint16_t raw_node_id;

    if (id < base) {
        return 0;
    }

    raw_node_id = (uint16_t)(id - base);
    if (raw_node_id > SIM_MOTOR_CAN_MAX_NODE_ID) {
        return 0;
    }

    *frame_node_id = (uint8_t)raw_node_id;
    return 1;
}

static int frame_matches_node(uint8_t node_id, uint8_t frame_node_id)
{
    return frame_node_id == 0u || frame_node_id == node_id;
}

static int handle_control(sim_motor_can_node_t *node,
                          const sim_motor_can_frame_t *frame,
                          uint8_t frame_node_id)
{
    uint8_t command;
    uint8_t seq;
    uint8_t ack_result = SIM_MOTOR_CAN_ACK_OK;
    int32_t detail = 0;
    int tx_result = SIM_MOTOR_CAN_OK;

    if (frame->dlc != 2u) {
        if (frame_node_id != 0u) {
            (void)enqueue_ack(node, 0x10u, SIM_MOTOR_CAN_ACK_BAD_DLC, 0u, frame->dlc);
        }
        enter_fault(node, SIM_MOTOR_CAN_FAULT_PROTOCOL);
        return SIM_MOTOR_CAN_ERROR_BAD_FRAME;
    }

    command = frame->data[0];
    seq = frame->data[1];
    refresh_watchdog(node);

    switch (command) {
    case SIM_MOTOR_CAN_CMD_DISABLE:
        if (node->state == SIM_MOTOR_CAN_STATE_FAULT) {
            ack_result = SIM_MOTOR_CAN_ACK_BAD_STATE;
            detail = node->fault_code;
        } else {
            enter_stopping(node, SIM_MOTOR_CAN_STATE_DISABLED);
        }
        break;

    case SIM_MOTOR_CAN_CMD_ENABLE:
        if (node->state == SIM_MOTOR_CAN_STATE_FAULT) {
            ack_result = SIM_MOTOR_CAN_ACK_BAD_STATE;
            detail = node->fault_code;
        } else {
            sim_motor_enable(&node->motor, 1);
            node->state = SIM_MOTOR_CAN_STATE_READY;
            node->stop_target_state = SIM_MOTOR_CAN_STATE_READY;
        }
        break;

    case SIM_MOTOR_CAN_CMD_FAULT_RESET:
        sim_motor_reset_fault(&node->motor);
        sim_motor_enable(&node->motor, 0);
        sim_motor_set_target_rpm(&node->motor, 0.0f);
        node->fault_code = SIM_MOTOR_CAN_FAULT_NONE;
        node->state = SIM_MOTOR_CAN_STATE_DISABLED;
        node->stop_target_state = SIM_MOTOR_CAN_STATE_DISABLED;
        node->command_age_ms = 0u;
        node->watchdog_active = 0u;
        break;

    case SIM_MOTOR_CAN_CMD_QUICK_STOP:
        if (node->state == SIM_MOTOR_CAN_STATE_FAULT) {
            ack_result = SIM_MOTOR_CAN_ACK_BAD_STATE;
            detail = node->fault_code;
        } else {
            enter_stopping(node, SIM_MOTOR_CAN_STATE_DISABLED);
        }
        break;

    case SIM_MOTOR_CAN_CMD_INJECT_FAULT:
        enter_fault(node, SIM_MOTOR_CAN_FAULT_MANUAL);
        break;

    default:
        ack_result = SIM_MOTOR_CAN_ACK_BAD_COMMAND;
        detail = command;
        enter_fault(node, SIM_MOTOR_CAN_FAULT_PROTOCOL);
        break;
    }

    if (frame_node_id != 0u) {
        tx_result = enqueue_ack(node, command, ack_result, seq, detail);
        if (tx_result != SIM_MOTOR_CAN_OK) {
            return tx_result;
        }
    }

    return ack_result == SIM_MOTOR_CAN_ACK_OK ? SIM_MOTOR_CAN_OK : SIM_MOTOR_CAN_ERROR_BAD_FRAME;
}

static int handle_speed_command(sim_motor_can_node_t *node,
                                const sim_motor_can_frame_t *frame,
                                uint8_t frame_node_id)
{
    int32_t target_rpm_x100;
    uint8_t seq;
    uint8_t ack_result = SIM_MOTOR_CAN_ACK_OK;
    int32_t detail = 0;
    int tx_result;

    if (frame->dlc != 6u) {
        if (frame_node_id != 0u) {
            (void)enqueue_ack(node, 0x20u, SIM_MOTOR_CAN_ACK_BAD_DLC, 0u, frame->dlc);
        }
        enter_fault(node, SIM_MOTOR_CAN_FAULT_PROTOCOL);
        return SIM_MOTOR_CAN_ERROR_BAD_FRAME;
    }

    target_rpm_x100 = can_read_i32_le(&frame->data[0]);
    seq = frame->data[4];
    refresh_watchdog(node);

    if (node->state == SIM_MOTOR_CAN_STATE_FAULT ||
        node->state == SIM_MOTOR_CAN_STATE_DISABLED ||
        node->state == SIM_MOTOR_CAN_STATE_INIT) {
        ack_result = SIM_MOTOR_CAN_ACK_BAD_STATE;
        detail = (int32_t)node->state;
    } else {
        sim_motor_set_target_rpm(&node->motor, rpm_from_x100(target_rpm_x100));
        if (can_absf(sim_motor_get_requested_rpm(&node->motor)) > SIM_MOTOR_CAN_STOP_EPSILON_RPM) {
            node->state = SIM_MOTOR_CAN_STATE_RUNNING;
        } else {
            enter_stopping(node, SIM_MOTOR_CAN_STATE_READY);
        }
        detail = rpm_to_x100(sim_motor_get_requested_rpm(&node->motor));
    }

    if (frame_node_id != 0u) {
        tx_result = enqueue_ack(node, 0x20u, ack_result, seq, detail);
        if (tx_result != SIM_MOTOR_CAN_OK) {
            return tx_result;
        }
    }

    return ack_result == SIM_MOTOR_CAN_ACK_OK ? SIM_MOTOR_CAN_OK : SIM_MOTOR_CAN_ERROR_BAD_FRAME;
}

static int handle_query(sim_motor_can_node_t *node,
                        const sim_motor_can_frame_t *frame,
                        uint8_t frame_node_id)
{
    uint8_t query_type;
    uint8_t seq;
    int tx_result = SIM_MOTOR_CAN_OK;

    if (frame->dlc != 2u) {
        if (frame_node_id != 0u) {
            (void)enqueue_ack(node, 0x30u, SIM_MOTOR_CAN_ACK_BAD_DLC, 0u, frame->dlc);
        }
        enter_fault(node, SIM_MOTOR_CAN_FAULT_PROTOCOL);
        return SIM_MOTOR_CAN_ERROR_BAD_FRAME;
    }

    query_type = frame->data[0];
    seq = frame->data[1];

    if (frame_node_id == 0u) {
        return SIM_MOTOR_CAN_OK;
    }

    if (query_type == SIM_MOTOR_CAN_QUERY_STATUS) {
        tx_result = enqueue_status(node);
    } else if (query_type == SIM_MOTOR_CAN_QUERY_SPEED) {
        tx_result = enqueue_speed(node);
    } else if (query_type == SIM_MOTOR_CAN_QUERY_PROTOCOL_INFO) {
        tx_result = enqueue_protocol_info(node);
    } else if (query_type == SIM_MOTOR_CAN_QUERY_POSITION) {
        tx_result = enqueue_position(node);
    } else {
        tx_result = enqueue_ack(node, 0x30u, SIM_MOTOR_CAN_ACK_BAD_COMMAND, seq, query_type);
    }

    return tx_result;
}

static int handle_heartbeat(sim_motor_can_node_t *node,
                            const sim_motor_can_frame_t *frame,
                            uint8_t frame_node_id)
{
    if (frame->dlc != 2u) {
        if (frame_node_id != 0u) {
            (void)enqueue_ack(node, 0x08u, SIM_MOTOR_CAN_ACK_BAD_DLC, 0u, frame->dlc);
        }
        enter_fault(node, SIM_MOTOR_CAN_FAULT_PROTOCOL);
        return SIM_MOTOR_CAN_ERROR_BAD_FRAME;
    }

    refresh_watchdog(node);
    return SIM_MOTOR_CAN_OK;
}

int sim_motor_can_node_init(sim_motor_can_node_t *node,
                            const sim_motor_can_node_config_t *config)
{
    if (node == 0 || config == 0) {
        return SIM_MOTOR_CAN_ERROR_NULL;
    }

    if (config->node_id == 0u || config->node_id > SIM_MOTOR_CAN_MAX_NODE_ID) {
        return SIM_MOTOR_CAN_ERROR_BAD_CONFIG;
    }

    node->config = *config;
    node->state = SIM_MOTOR_CAN_STATE_INIT;
    node->stop_target_state = SIM_MOTOR_CAN_STATE_DISABLED;
    node->fault_code = SIM_MOTOR_CAN_FAULT_NONE;
    node->uptime_ms = 0u;
    node->command_age_ms = 0u;
    node->status_elapsed_ms = 0u;
    node->speed_elapsed_ms = 0u;
    node->watchdog_active = 0u;
    node->tx_head = 0u;
    node->tx_tail = 0u;
    node->tx_count = 0u;

    sim_motor_init(&node->motor, &config->motor_config);

    return SIM_MOTOR_CAN_OK;
}

int sim_motor_can_node_receive(sim_motor_can_node_t *node,
                               const sim_motor_can_frame_t *frame)
{
    uint8_t frame_node_id = 0u;

    if (node == 0 || frame == 0) {
        return SIM_MOTOR_CAN_ERROR_NULL;
    }

    if (frame->dlc > 8u || frame->id > 0x7FFu) {
        return SIM_MOTOR_CAN_ERROR_BAD_FRAME;
    }

    if (parse_frame_node(frame->id, SIM_MOTOR_CAN_ID_HEARTBEAT_BASE, &frame_node_id) &&
        frame->id <= (SIM_MOTOR_CAN_ID_HEARTBEAT_BASE + SIM_MOTOR_CAN_MAX_NODE_ID)) {
        if (!frame_matches_node(node->config.node_id, frame_node_id)) {
            return SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE;
        }
        return handle_heartbeat(node, frame, frame_node_id);
    }

    if (parse_frame_node(frame->id, SIM_MOTOR_CAN_ID_CONTROL_BASE, &frame_node_id) &&
        frame->id <= (SIM_MOTOR_CAN_ID_CONTROL_BASE + SIM_MOTOR_CAN_MAX_NODE_ID)) {
        if (!frame_matches_node(node->config.node_id, frame_node_id)) {
            return SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE;
        }
        return handle_control(node, frame, frame_node_id);
    }

    if (parse_frame_node(frame->id, SIM_MOTOR_CAN_ID_SPEED_COMMAND_BASE, &frame_node_id) &&
        frame->id <= (SIM_MOTOR_CAN_ID_SPEED_COMMAND_BASE + SIM_MOTOR_CAN_MAX_NODE_ID)) {
        if (!frame_matches_node(node->config.node_id, frame_node_id)) {
            return SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE;
        }
        return handle_speed_command(node, frame, frame_node_id);
    }

    if (parse_frame_node(frame->id, SIM_MOTOR_CAN_ID_QUERY_BASE, &frame_node_id) &&
        frame->id <= (SIM_MOTOR_CAN_ID_QUERY_BASE + SIM_MOTOR_CAN_MAX_NODE_ID)) {
        if (!frame_matches_node(node->config.node_id, frame_node_id)) {
            return SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE;
        }
        return handle_query(node, frame, frame_node_id);
    }

    return SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE;
}

void sim_motor_can_node_update(sim_motor_can_node_t *node,
                               float dt_s,
                               uint32_t dt_ms)
{
    if (node == 0) {
        return;
    }

    node->uptime_ms += dt_ms;

    if (node->watchdog_active && node->config.command_timeout_ms > 0u &&
        node->state != SIM_MOTOR_CAN_STATE_FAULT) {
        node->command_age_ms += dt_ms;
        if (node->command_age_ms > node->config.command_timeout_ms) {
            enter_fault(node, SIM_MOTOR_CAN_FAULT_COMM_TIMEOUT);
        }
    }

    sim_motor_update(&node->motor, dt_s);
    update_state_from_motion(node);

    if (node->config.status_period_ms > 0u) {
        node->status_elapsed_ms += dt_ms;
        if (node->status_elapsed_ms >= node->config.status_period_ms) {
            node->status_elapsed_ms = 0u;
            (void)enqueue_status(node);
        }
    }

    if (node->config.speed_period_ms > 0u) {
        node->speed_elapsed_ms += dt_ms;
        if (node->speed_elapsed_ms >= node->config.speed_period_ms) {
            node->speed_elapsed_ms = 0u;
            (void)enqueue_speed(node);
        }
    }
}

int sim_motor_can_node_next_tx(sim_motor_can_node_t *node,
                               sim_motor_can_frame_t *out_frame)
{
    if (node == 0 || out_frame == 0) {
        return SIM_MOTOR_CAN_ERROR_NULL;
    }

    if (node->tx_count == 0u) {
        return SIM_MOTOR_CAN_ERROR_TX_EMPTY;
    }

    *out_frame = node->tx_queue[node->tx_head];
    node->tx_head = (uint8_t)((node->tx_head + 1u) % SIM_MOTOR_CAN_TX_QUEUE_SIZE);
    node->tx_count--;

    return SIM_MOTOR_CAN_OK;
}

sim_motor_can_state_t sim_motor_can_node_get_state(const sim_motor_can_node_t *node)
{
    return node != 0 ? node->state : SIM_MOTOR_CAN_STATE_FAULT;
}

uint16_t sim_motor_can_node_get_fault_code(const sim_motor_can_node_t *node)
{
    return node != 0 ? node->fault_code : SIM_MOTOR_CAN_FAULT_PROTOCOL;
}

const sim_motor_t *sim_motor_can_node_get_motor(const sim_motor_can_node_t *node)
{
    return node != 0 ? &node->motor : 0;
}
