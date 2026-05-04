# Agent Instructions

## Project overview
- Portable C99 simulated motor driver for PC, bare-metal MCU, or RTOS.
- Caller owns time; advance with periodic sim_motor_update() calls.
- No dynamic memory, threads, or timers.

## Build and test
- Configure: cmake -S . -B build -G Ninja
- Build: cmake --build build
- Test: ctest --test-dir build
- Run PC example: ./build/sim_motor_example_pc(.exe on Windows)

## Key paths
- Public API headers: [include/sim_motor/](../include/sim_motor/)
- Implementations: [src/](../src/)
- Unit tests: [tests/](../tests/)
- PC example: [examples/sim_motor_example_pc.c](../examples/sim_motor_example_pc.c)
- CAN protocol spec: [docs/CAN_PROTOCOL.md](../docs/CAN_PROTOCOL.md)
- Zephyr integration: [zephyr/](../zephyr/) (module.yml, Kconfig)

## Conventions and pitfalls
- All APIs use float; CAN encoding uses int32 rpm x100 (little-endian).
- Requested -> setpoint -> actual RPM are ramp-limited; avoid jumping values in tests.
- sim_motor_can is transport-agnostic; application owns frame I/O and tx queue polling.
- CAN watchdog: heartbeats/control/speed command reset timeout; timeout triggers fault.

## Reference docs
- [README.md](../README.md) for usage and CMake options (SIM_MOTOR_BUILD_EXAMPLES/TESTS).
- [docs/CAN_PROTOCOL.md](../docs/CAN_PROTOCOL.md) for message IDs, states, and encoding details.
