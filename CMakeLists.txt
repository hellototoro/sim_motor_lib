cmake_minimum_required(VERSION 3.16)

project(sim_motor_lib
    VERSION 0.1.0
    DESCRIPTION "Portable simulated motor driver speed-loop library"
    LANGUAGES C)

option(SIM_MOTOR_BUILD_EXAMPLES "Build PC simulation examples" ON)
option(SIM_MOTOR_BUILD_TESTS "Build unit tests" ON)

add_library(sim_motor STATIC
    src/sim_motor.c)

target_include_directories(sim_motor
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

target_compile_features(sim_motor PUBLIC c_std_99)

if(MSVC)
    target_compile_options(sim_motor PRIVATE /W4)
else()
    target_compile_options(sim_motor PRIVATE -Wall -Wextra -Wpedantic)
endif()

if(SIM_MOTOR_BUILD_EXAMPLES)
    add_executable(sim_motor_example_pc
        examples/sim_motor_example_pc.c)
    target_link_libraries(sim_motor_example_pc PRIVATE sim_motor)
endif()

if(SIM_MOTOR_BUILD_TESTS)
    enable_testing()

    add_executable(sim_motor_tests
        tests/test_sim_motor.c)
    target_link_libraries(sim_motor_tests PRIVATE sim_motor)

    add_test(NAME sim_motor_tests COMMAND sim_motor_tests)
endif()
