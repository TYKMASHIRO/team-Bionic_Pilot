# 测试：GoogleTest
# 单元测试（始终构建）；硬件测试仅在 ENABLE_HARDWARE_TESTS=ON 时构建。

set(GTEST_LIB gtest_main)
if(TARGET GTest::gtest_main)
    set(GTEST_LIB GTest::gtest_main)
endif()

function(robot_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE robotics_core mock_devices ${GTEST_LIB})
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/src)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# ---- 单元测试 ----
robot_add_test(unit_mock_connect unit/mock_connect_test.cpp)
robot_add_test(unit_state_snapshot unit/state_snapshot_test.cpp)
robot_add_test(unit_safety_reject unit/safety_reject_test.cpp)
robot_add_test(unit_command_cancel unit/command_cancel_test.cpp)
robot_add_test(unit_config_validation unit/config_validation_test.cpp)
robot_add_test(unit_error_model unit/error_model_test.cpp)

# ---- 集成测试（Mock 双设备）----
robot_add_test(integration_mock_double integration/mock_double_test.cpp)

# ---- 硬件测试（需真实设备，默认不构建）----
if(ENABLE_HARDWARE_TESTS)
    add_executable(hw_arm_connect hardware/hw_arm_connect_test.cpp)
    target_link_libraries(hw_arm_connect PRIVATE robotics_core mock_devices)
    # 硬件测试默认不加入 ctest（显式运行）
    set_tests_properties(hw_arm_connect PROPERTIES DISABLED TRUE)
endif()
