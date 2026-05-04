# sim_motor_lib

用于在 PC、裸机 MCU 或 RTOS 项目中进行速度环测试的可移植 C99 模拟电机驱动。

该库不使用动态内存、线程或定时器，也不依赖硬件。调用方负责时间推进：周期性
调用 `sim_motor_update()` 并传入以秒为单位的时间增量。

## 模型

速度指令分为三层：

- `requested_rpm`：用户请求的目标转速，会被限制在配置的速度范围内。
- `setpoint_rpm`：驱动内部设定值，以斜坡限幅方式向请求转速逼近。
- `actual_rpm`：模拟测得的电机转速，以斜坡限幅方式向设定值逼近。
- `position_rev`：由实际转速积分得到的有符号电机轴位置，单位为圈。

当电机被禁用或故障时，设定值会以斜坡回到 `0 rpm`。实际转速仍按配置的加速度或
减速度限制，继续跟随设定值变化。

当实际速度到达内部设定值后，`actual_rpm` 会按 `steady_noise_ratio` 产生小范围
随机浮动。配置值为 `0` 时使用默认 `0.01`，即目标速度的 `±1%`；配置为负数可关闭
浮动。随机序列由库内部 PRNG 产生，可通过 `sim_motor_set_noise_seed()` 固定。

## 构建

```sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
```

运行 PC 示例：

```sh
./build/sim_motor_example_pc
```

在 Windows 上使用 Ninja 时，可执行文件通常为：

```sh
./build/sim_motor_example_pc.exe
```

示例输出 CSV：

```text
time,requested,setpoint,actual,position_rev,enabled,fault
```

## CAN 协议层

可选的 `sim_motor_can` 库在电机模型之上提供可移植的 CAN 2.0A 协议层。它包含
标准帧解析、模拟驱动状态机、指令看门狗超时、固定大小发送队列以及周期性
状态/速度反馈帧。

CAN 层不会直接调用 MCU HAL。将接收到的帧喂给
`sim_motor_can_node_receive()`，在周期任务中调用
`sim_motor_can_node_update()`，并通过 `sim_motor_can_node_next_tx()`
取出待发送帧。

完整的中文协议文档：

- `docs/CAN_PROTOCOL.md`

## MCU 使用

将以下文件复制或添加到固件工程：

- `include/sim_motor/sim_motor.h`
- `src/sim_motor.c`

然后在固定周期循环、定时器回调或 RTOS 任务中调用 `sim_motor_update()`：

```c
sim_motor_t motor;

const sim_motor_config_t config = {
    1500.0f,
    2000.0f,
    900.0f,
    1200.0f,
    3000.0f,
    0.01f,
};

sim_motor_init(&motor, &config);
sim_motor_enable(&motor, 1);
sim_motor_set_target_rpm(&motor, 1000.0f);

/* 每 10 ms 调用一次。 */
sim_motor_update(&motor, 0.01f);

/* 读取有符号电机轴位置，单位为圈。 */
float position_rev = sim_motor_get_position_rev(&motor);
```

## Zephyr 使用

该仓库可直接作为 Zephyr 模块使用:

```shell
west build -p always -b <board> app -- -DEXTRA_ZEPHYR_MODULES="path/to/sim_motor_lib"
```

在 `prj.conf` 中启用库：

```text
CONFIG_SIM_MOTOR_LIB=y
CONFIG_SIM_MOTOR_CAN_NODE_ID=5
CONFIG_SIM_MOTOR_CAN_BITRATE=500000
```
