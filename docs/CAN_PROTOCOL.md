# 模拟电机驱动器 CAN 总线通信说明书

## 1. 概述

本文档定义 `sim_motor_can` 模块使用的 CAN 2.0A 通信协议。该协议用于在 PC、裸机 MCU 或 RTOS 环境中控制模拟电机驱动器，支持多节点、速度控制、状态查询、周期反馈、通信超时保护和基础故障恢复。

协议层只处理 CAN 帧编解码与驱动器状态机，不绑定任何 CAN HAL、RTOS、线程或系统时钟。应用层负责从真实 CAN 外设接收帧并调用 `sim_motor_can_node_receive()`，再通过 `sim_motor_can_node_next_tx()` 取出待发送帧交给 CAN 外设发送。

建议总线速率为 `500 kbit/s` 或 `1 Mbit/s`。当前协议仅支持 Classical CAN 标准帧，数据区最大 8 字节，不支持 CAN FD。

## 2. 基本约定

- 帧格式：CAN 2.0A 标准帧，11 位 ID。
- 节点地址：`node_id = 1..127`。
- 广播地址：`node_id = 0`，广播命令会被所有节点执行，但不会产生应答。
- 字节序：所有多字节字段均为 little-endian。
- 速度单位：`rpm_x100`，类型为 `int32_t`。
  - `1234.56 rpm` 编码为 `123456`。
  - `-500.00 rpm` 编码为 `-50000`。
- 通信超时：节点收到合法 Heartbeat、Control 或 Speed Command 后启动/刷新 watchdog；超过配置时间未收到这些帧时进入故障并平滑停机。

## 3. CAN ID 分配

| 方向 | CAN ID | 名称 | DLC | 说明 |
| --- | --- | --- | --- | --- |
| 主站到驱动器 | `0x080 + node_id` | Heartbeat | 2 | 主站心跳，刷新通信 watchdog |
| 主站到驱动器 | `0x100 + node_id` | Control | 2 | 使能、禁用、复位故障、急停、注入故障 |
| 主站到驱动器 | `0x200 + node_id` | Speed Command | 6 | 设置目标速度 |
| 主站到驱动器 | `0x300 + node_id` | Query | 2 | 查询状态、速度或协议信息 |
| 驱动器到主站 | `0x500 + node_id` | Status | 8 | 驱动器状态周期上报或查询响应 |
| 驱动器到主站 | `0x580 + node_id` | Speed Feedback | 8 | 当前速度和内部设定速度 |
| 驱动器到主站 | `0x5C0 + node_id` | Position Feedback | 8 | 当前有符号电机轴位置 |
| 驱动器到主站 | `0x600 + node_id` | Ack/Error | 8 | 命令确认、错误或协议信息 |

示例：节点 `5` 的速度命令帧 ID 为 `0x205`，状态反馈帧 ID 为 `0x505`。

## 4. 主站发送帧

### 4.1 Heartbeat

CAN ID：`0x080 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | seq | `uint8_t` | 主站序号 |
| 1 | flags | `uint8_t` | 预留，当前填 0 |

收到合法 Heartbeat 后，驱动器刷新通信 watchdog。Heartbeat 不产生应答。

### 4.2 Control

CAN ID：`0x100 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | command | `uint8_t` | 控制命令 |
| 1 | seq | `uint8_t` | 主站序号 |

命令定义：

| command | 名称 | 行为 |
| --- | --- | --- |
| 0 | disable | 禁用输出，速度平滑降到 0 |
| 1 | enable | 进入 READY，可接收速度命令 |
| 2 | fault_reset | 清故障，回到 DISABLED |
| 3 | quick_stop | 急停到 DISABLED，仍按减速度曲线停机 |
| 4 | inject_fault | 手动注入故障，用于测试 |

非广播 Control 会产生 Ack/Error。广播 Control 会执行但不应答。

### 4.3 Speed Command

CAN ID：`0x200 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0..3 | target_rpm_x100 | `int32_t` | 目标速度，little-endian |
| 4 | seq | `uint8_t` | 主站序号 |
| 5 | flags | `uint8_t` | 预留，当前填 0 |

Speed Command 只在 READY、RUNNING 或 STOPPING 状态下接受。速度会被限制在电机配置的最大速度范围内，内部设定速度和实际反馈速度不会突变，而是按配置的加速度/减速度限制变化。

### 4.4 Query

CAN ID：`0x300 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | query_type | `uint8_t` | 查询类型 |
| 1 | seq | `uint8_t` | 主站序号 |

查询类型：

| query_type | 名称 | 响应 |
| --- | --- | --- |
| 1 | status | Status |
| 2 | speed | Speed Feedback |
| 3 | protocol_info | Ack/Error 格式的协议信息 |
| 4 | position | Position Feedback |

广播 Query 不产生响应。

## 5. 驱动器发送帧

### 5.1 Status

CAN ID：`0x500 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | state | `uint8_t` | 当前状态机状态 |
| 1 | flags | `uint8_t` | 状态标志 |
| 2..3 | fault_code | `uint16_t` | 故障码，little-endian |
| 4..7 | uptime_ms | `uint32_t` | 节点运行时间，little-endian |

状态值：

| state | 名称 |
| --- | --- |
| 0 | INIT |
| 1 | DISABLED |
| 2 | READY |
| 3 | RUNNING |
| 4 | STOPPING |
| 5 | FAULT |

flags 位定义：

| bit | 掩码 | 名称 | 说明 |
| --- | --- | --- | --- |
| 0 | `0x01` | enabled | 电机输出已使能 |
| 1 | `0x02` | faulted | 当前处于故障 |
| 2 | `0x04` | moving | 设定速度或实际速度非 0 |
| 3 | `0x08` | watchdog_active | 通信 watchdog 已启动 |

故障码：

| fault_code | 名称 | 说明 |
| --- | --- | --- |
| 0 | NONE | 无故障 |
| 1 | MANUAL | 手动注入故障 |
| 2 | COMM_TIMEOUT | 通信超时 |
| 3 | PROTOCOL | 协议错误 |

### 5.2 Speed Feedback

CAN ID：`0x580 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0..3 | actual_rpm_x100 | `int32_t` | 模拟读取到的当前速度 |
| 4..7 | setpoint_rpm_x100 | `int32_t` | 驱动器内部速度设定值 |

`actual_rpm_x100` 和 `setpoint_rpm_x100` 均为 little-endian。

### 5.3 Position Feedback

CAN ID：`0x5C0 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0..7 | position_rev_x1000 | `int64_t` | 有符号电机轴位置，单位为圈，乘以 1000 |

`position_rev_x1000` 为 little-endian。正转位置增加，反转位置减少。例如
`12.345 rev` 编码为 `12345`，`-1.000 rev` 编码为 `-1000`。

### 5.4 Ack/Error

CAN ID：`0x600 + node_id`

| Byte | 字段 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0 | source_cmd | `uint8_t` | 来源命令或查询类型 |
| 1 | result | `uint8_t` | 结果码 |
| 2 | state | `uint8_t` | 当前状态 |
| 3 | seq | `uint8_t` | 回显主站序号 |
| 4..7 | detail | `int32_t` | 附加信息 |

结果码：

| result | 名称 | 说明 |
| --- | --- | --- |
| 0 | OK | 成功 |
| 1 | BAD_DLC | DLC 不符合协议 |
| 2 | BAD_COMMAND | 命令或查询类型非法 |
| 3 | BAD_STATE | 当前状态不允许该命令 |
| 4 | TX_FULL | 内部发送队列满 |

Speed Command 成功时，`detail` 返回限幅后的 `requested_rpm_x100`。

## 6. 状态机

状态迁移：

```text
INIT -> DISABLED
DISABLED --enable--> READY
READY --speed != 0--> RUNNING
RUNNING --speed = 0--> STOPPING -> READY
READY/RUNNING --disable or quick_stop--> STOPPING -> DISABLED
ANY --manual fault/protocol error/comm timeout--> FAULT
FAULT --fault_reset--> DISABLED
```

说明：

- DISABLED：输出禁用，速度目标为 0。
- READY：输出使能，等待速度命令。
- RUNNING：正在跟踪非零目标速度。
- STOPPING：正在按减速度曲线停机。
- FAULT：故障态，拒绝 enable 和 speed command，只允许 fault reset。

## 7. 主站推荐流程

1. 周期发送 Heartbeat，周期应小于驱动器配置的 `command_timeout_ms`。
2. 发送 Control enable，等待 Ack/Error 成功。
3. 周期发送 Speed Command，或在目标速度变化时发送。
4. 周期接收 Status 和 Speed Feedback，按需查询 Position Feedback。
5. 如 Status 显示 FAULT，发送 Control fault_reset。
6. reset 成功后重新 enable，再恢复速度命令。

## 8. MCU 接入示例

```c
static sim_motor_can_node_t node;

void app_init(void)
{
    sim_motor_can_node_config_t cfg = {
        5,
        100,
        20,
        20,
        {1500.0f, 2000.0f, 900.0f, 1200.0f, 3000.0f, 0.01f}
    };

    sim_motor_can_node_init(&node, &cfg);
}

void can_rx_callback(uint16_t id, const uint8_t *data, uint8_t dlc)
{
    sim_motor_can_frame_t frame;
    unsigned int i;

    frame.id = id;
    frame.dlc = dlc;
    for (i = 0; i < 8u; ++i) {
        frame.data[i] = i < dlc ? data[i] : 0u;
    }

    (void)sim_motor_can_node_receive(&node, &frame);
}

void app_10ms_task(void)
{
    sim_motor_can_frame_t tx;

    sim_motor_can_node_update(&node, 0.01f, 10u);

    while (sim_motor_can_node_next_tx(&node, &tx) == SIM_MOTOR_CAN_OK) {
        can_hal_send(tx.id, tx.data, tx.dlc);
    }
}
```

该示例中的 `can_hal_send()` 由具体 MCU 平台提供，协议库本身不依赖任何 HAL。
