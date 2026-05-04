#ifndef SIM_MOTOR_SIM_MOTOR_CAN_H
#define SIM_MOTOR_SIM_MOTOR_CAN_H

#include "sim_motor/sim_motor.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SIM_MOTOR_CAN_TX_QUEUE_SIZE 8u
#define SIM_MOTOR_CAN_MAX_NODE_ID 127u

#define SIM_MOTOR_CAN_ID_HEARTBEAT_BASE 0x080u
#define SIM_MOTOR_CAN_ID_CONTROL_BASE 0x100u
#define SIM_MOTOR_CAN_ID_SPEED_COMMAND_BASE 0x200u
#define SIM_MOTOR_CAN_ID_QUERY_BASE 0x300u
#define SIM_MOTOR_CAN_ID_STATUS_BASE 0x500u
#define SIM_MOTOR_CAN_ID_SPEED_FEEDBACK_BASE 0x580u
#define SIM_MOTOR_CAN_ID_POSITION_FEEDBACK_BASE 0x5C0u
#define SIM_MOTOR_CAN_ID_ACK_BASE 0x600u

typedef struct {
    uint16_t id;
    uint8_t dlc;
    uint8_t data[8];
} sim_motor_can_frame_t;

typedef enum {
    SIM_MOTOR_CAN_OK = 0,
    SIM_MOTOR_CAN_ERROR_NULL = -1,
    SIM_MOTOR_CAN_ERROR_BAD_CONFIG = -2,
    SIM_MOTOR_CAN_ERROR_BAD_FRAME = -3,
    SIM_MOTOR_CAN_ERROR_NOT_FOR_NODE = -4,
    SIM_MOTOR_CAN_ERROR_TX_FULL = -5,
    SIM_MOTOR_CAN_ERROR_TX_EMPTY = -6
} sim_motor_can_result_t;

typedef enum {
    SIM_MOTOR_CAN_STATE_INIT = 0,
    SIM_MOTOR_CAN_STATE_DISABLED = 1,
    SIM_MOTOR_CAN_STATE_READY = 2,
    SIM_MOTOR_CAN_STATE_RUNNING = 3,
    SIM_MOTOR_CAN_STATE_STOPPING = 4,
    SIM_MOTOR_CAN_STATE_FAULT = 5
} sim_motor_can_state_t;

typedef enum {
    SIM_MOTOR_CAN_CMD_DISABLE = 0,
    SIM_MOTOR_CAN_CMD_ENABLE = 1,
    SIM_MOTOR_CAN_CMD_FAULT_RESET = 2,
    SIM_MOTOR_CAN_CMD_QUICK_STOP = 3,
    SIM_MOTOR_CAN_CMD_INJECT_FAULT = 4
} sim_motor_can_command_t;

typedef enum {
    SIM_MOTOR_CAN_QUERY_STATUS = 1,
    SIM_MOTOR_CAN_QUERY_SPEED = 2,
    SIM_MOTOR_CAN_QUERY_PROTOCOL_INFO = 3,
    SIM_MOTOR_CAN_QUERY_POSITION = 4
} sim_motor_can_query_t;

typedef enum {
    SIM_MOTOR_CAN_ACK_OK = 0,
    SIM_MOTOR_CAN_ACK_BAD_DLC = 1,
    SIM_MOTOR_CAN_ACK_BAD_COMMAND = 2,
    SIM_MOTOR_CAN_ACK_BAD_STATE = 3,
    SIM_MOTOR_CAN_ACK_TX_FULL = 4
} sim_motor_can_ack_result_t;

typedef enum {
    SIM_MOTOR_CAN_FAULT_NONE = 0,
    SIM_MOTOR_CAN_FAULT_MANUAL = 1,
    SIM_MOTOR_CAN_FAULT_COMM_TIMEOUT = 2,
    SIM_MOTOR_CAN_FAULT_PROTOCOL = 3
} sim_motor_can_fault_code_t;

typedef struct {
    uint8_t node_id;
    uint32_t command_timeout_ms;
    uint32_t status_period_ms;
    uint32_t speed_period_ms;
    sim_motor_config_t motor_config;
} sim_motor_can_node_config_t;

typedef struct {
    sim_motor_t motor;
    sim_motor_can_node_config_t config;
    sim_motor_can_state_t state;
    sim_motor_can_state_t stop_target_state;
    uint16_t fault_code;
    uint32_t uptime_ms;
    uint32_t command_age_ms;
    uint32_t status_elapsed_ms;
    uint32_t speed_elapsed_ms;
    uint8_t watchdog_active;
    sim_motor_can_frame_t tx_queue[SIM_MOTOR_CAN_TX_QUEUE_SIZE];
    uint8_t tx_head;
    uint8_t tx_tail;
    uint8_t tx_count;
} sim_motor_can_node_t;

int sim_motor_can_node_init(sim_motor_can_node_t *node,
                            const sim_motor_can_node_config_t *config);
int sim_motor_can_node_receive(sim_motor_can_node_t *node,
                               const sim_motor_can_frame_t *frame);
void sim_motor_can_node_update(sim_motor_can_node_t *node,
                               float dt_s,
                               uint32_t dt_ms);
int sim_motor_can_node_next_tx(sim_motor_can_node_t *node,
                               sim_motor_can_frame_t *out_frame);

sim_motor_can_state_t sim_motor_can_node_get_state(const sim_motor_can_node_t *node);
uint16_t sim_motor_can_node_get_fault_code(const sim_motor_can_node_t *node);
const sim_motor_t *sim_motor_can_node_get_motor(const sim_motor_can_node_t *node);

#ifdef __cplusplus
}
#endif

#endif
