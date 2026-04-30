# sim_motor_lib

Portable C99 simulated motor driver for speed-loop testing on a PC, bare-metal
MCU, or RTOS project.

The library has no dynamic allocation, no threads, no timers, and no hardware
dependencies. The caller owns time by periodically calling `sim_motor_update()`
with the elapsed time in seconds.

## Model

The speed command is modeled in three layers:

- `requested_rpm`: user-requested target speed, clamped to the configured speed
  range.
- `setpoint_rpm`: internal driver setpoint, ramp-limited toward the requested
  speed.
- `actual_rpm`: simulated measured motor speed, ramp-limited toward the
  setpoint.

Disabled or faulted motors ramp the setpoint back to `0 rpm`, while the actual
speed continues to follow the setpoint using the configured motor acceleration
or deceleration limits.

## Build

```sh
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
```

Run the PC example:

```sh
./build/sim_motor_example_pc
```

On Windows with Ninja, the executable is usually:

```sh
./build/sim_motor_example_pc.exe
```

The example prints CSV:

```text
time,requested,setpoint,actual,enabled,fault
```

## CAN Protocol Layer

The optional `sim_motor_can` library adds a portable CAN 2.0A protocol layer on
top of the motor model. It handles standard-frame parsing, a simulated driver
state machine, command watchdog timeout, fixed-size transmit queue, and periodic
status/speed feedback frames.

The CAN layer does not call any MCU HAL directly. Feed received frames into
`sim_motor_can_node_receive()`, call `sim_motor_can_node_update()` from your
periodic task, and drain outgoing frames with `sim_motor_can_node_next_tx()`.

See the full Chinese protocol document:

- `docs/CAN_PROTOCOL.md`

## MCU Usage

Copy or add these files to the firmware project:

- `include/sim_motor/sim_motor.h`
- `src/sim_motor.c`

Then call `sim_motor_update()` from a fixed-period loop, timer callback, or RTOS
task:

```c
sim_motor_t motor;

const sim_motor_config_t config = {
    1500.0f,
    2000.0f,
    900.0f,
    1200.0f,
    3000.0f,
};

sim_motor_init(&motor, &config);
sim_motor_enable(&motor, 1);
sim_motor_set_target_rpm(&motor, 1000.0f);

/* Called every 10 ms. */
sim_motor_update(&motor, 0.01f);
```
