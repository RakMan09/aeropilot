# FreeRTOS kernel + AeroPilot board port for the QEMU mps2-an385 (Cortex-M3).

set(FREERTOS_KERNEL ${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-Kernel)
set(FREERTOS_PORT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/port)

add_library(freertos_port STATIC
    ${FREERTOS_KERNEL}/tasks.c
    ${FREERTOS_KERNEL}/queue.c
    ${FREERTOS_KERNEL}/list.c
    ${FREERTOS_KERNEL}/timers.c
    ${FREERTOS_KERNEL}/event_groups.c
    ${FREERTOS_KERNEL}/stream_buffer.c
    ${FREERTOS_KERNEL}/portable/MemMang/heap_4.c
    ${FREERTOS_KERNEL}/portable/GCC/ARM_CM3/port.c
)

target_include_directories(freertos_port PUBLIC
    ${FREERTOS_KERNEL}/include
    ${FREERTOS_KERNEL}/portable/GCC/ARM_CM3
    ${FREERTOS_PORT_DIR}
)

# The kernel/port compile cleanly; keep our own warnings for the port glue but
# avoid failing the build on third-party kernel warnings.
target_compile_options(freertos_port PRIVATE -Wall)
