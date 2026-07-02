# Host-side Unity test suite for the portable AeroPilot core.

add_library(unity STATIC Unity/src/unity.c)
target_include_directories(unity PUBLIC Unity/src)
target_compile_definitions(unity PUBLIC UNITY_INCLUDE_DOUBLE UNITY_INCLUDE_FLOAT)

set(AEROPILOT_TESTS
    test_crc16
    test_frame
    test_arinc429
    test_bus
    test_kalman
    test_complementary
    test_fusion
    test_flight_sm
    test_watchdog
    test_sim
    test_scenario
)

foreach(test_name ${AEROPILOT_TESTS})
    add_executable(${test_name} ${test_name}.c)
    target_link_libraries(${test_name} PRIVATE aeropilot_core unity)
    target_compile_options(${test_name} PRIVATE -Wall -Wextra)
    add_test(NAME ${test_name} COMMAND ${test_name})
endforeach()
