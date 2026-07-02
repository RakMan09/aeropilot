cmake_minimum_required(VERSION 3.16)

project(aeropilot C ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

# AEROPILOT_BUILD selects what to build:
#   firmware    - the ARM Cortex-M3 flight image for QEMU (needs the toolchain
#                 file, see cmake/arm-none-eabi.cmake)
#   host_tests  - the portable core + Unity host test suite (native gcc)
if(NOT DEFINED AEROPILOT_BUILD)
    if(CMAKE_CROSSCOMPILING)
        set(AEROPILOT_BUILD "firmware")
    else()
        set(AEROPILOT_BUILD "host_tests")
    endif()
endif()

message(STATUS "AeroPilot build type: ${AEROPILOT_BUILD}")

# ---------------------------------------------------------------------------
# Portable core library: physics sim, fusion, protocol, state machine.
# Contains no RTOS dependency so it links into both the firmware and tests.
# ---------------------------------------------------------------------------
set(AEROPILOT_CORE_SOURCES
    src/aeropilot.c
    src/sim/rocket_sim.c
    src/fusion/kalman.c
    src/fusion/complementary.c
    src/fusion/fusion.c
    src/state/flight_sm.c
    src/proto/crc16.c
    src/proto/frame.c
    src/proto/arinc429.c
    src/proto/bus.c
    src/watchdog/watchdog.c
)

add_library(aeropilot_core STATIC ${AEROPILOT_CORE_SOURCES})
target_include_directories(aeropilot_core PUBLIC src)
target_compile_options(aeropilot_core PRIVATE -Wall -Wextra)
if(NOT CMAKE_CROSSCOMPILING)
    target_link_libraries(aeropilot_core PUBLIC m)
endif()

if(AEROPILOT_BUILD STREQUAL "firmware")
    add_subdirectory(freertos)
    add_executable(aeropilot.elf
        src/app/main.c
        src/tasks/sensor_task.c
        src/tasks/fusion_task.c
        src/tasks/state_machine_task.c
        src/tasks/telemetry_task.c
        src/tasks/watchdog_task.c
        # Board port glue compiled directly into the image so our strong
        # startup/syscall symbols override the newlib nosys stubs.
        freertos/port/startup_gcc.c
        freertos/port/syscalls.c
        freertos/port/mps2_uart.c
    )
    target_link_libraries(aeropilot.elf PRIVATE aeropilot_core freertos_port)
    target_compile_options(aeropilot.elf PRIVATE -Wall -Wextra)

    # Optional watchdog fault-injection build: hang the fusion task shortly
    # after launch to prove the watchdog trips to SAFE.
    if(AEROPILOT_INJECT_HANG)
        target_compile_definitions(aeropilot.elf PRIVATE
            AEROPILOT_INJECT_HANG=${AEROPILOT_INJECT_HANG})
    endif()

    set(LINKER_SCRIPT ${CMAKE_SOURCE_DIR}/freertos/port/mps2_m3.ld)
    target_link_options(aeropilot.elf PRIVATE
        -T${LINKER_SCRIPT}
        -Wl,-Map=${CMAKE_BINARY_DIR}/aeropilot.map
        -Wl,--gc-sections
        # Pull in newlib-nano's floating-point printf support.
        -Wl,-u,_printf_float
    )
    set_target_properties(aeropilot.elf PROPERTIES LINK_DEPENDS ${LINKER_SCRIPT})

    # Link the math library provided by newlib for the fusion/sim floats.
    target_link_libraries(aeropilot.elf PRIVATE m)

    add_custom_command(TARGET aeropilot.elf POST_BUILD
        COMMAND ${CMAKE_SIZE} $<TARGET_FILE:aeropilot.elf>
        VERBATIM)
else()
    enable_testing()
    add_subdirectory(test)
endif()
