# 测试：GoogleTest
# 单元测试（始终构建）；硬件测试仅在 ENABLE_HARDWARE_TESTS=ON 时构建。

set(GTEST_LIB GTest::gtest_main)

function(robot_add_test name source)
    add_executable(${name} ${source})
    target_link_libraries(${name} PRIVATE robotics_core mock_devices ${GTEST_LIB})
    target_include_directories(${name} PRIVATE ${CMAKE_SOURCE_DIR}/src)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

# ---- 单元测试 ----
robot_add_test(unit_mock_connect unit/mock_connect_test.cpp)
robot_add_test(unit_state_snapshot unit/state_snapshot_test.cpp)
robot_add_test(unit_state_collector unit/state_collector_test.cpp)
robot_add_test(unit_state_store_seq unit/state_store_seq_test.cpp)
robot_add_test(unit_recorder unit/recorder_test.cpp)
robot_add_test(unit_csv_sink unit/csv_sink_test.cpp)
robot_add_test(unit_safety_reject unit/safety_reject_test.cpp)
robot_add_test(unit_command_cancel unit/command_cancel_test.cpp)
robot_add_test(unit_config_validation unit/config_validation_test.cpp)
robot_add_test(unit_error_model unit/error_model_test.cpp)

# ---- RM 错误映射单元测试（无硬件，验证转换逻辑）----
if(ENABLE_REALMAN_DRIVER AND TARGET realman_driver)
    robot_add_test(unit_rm_error_map unit/rm_error_map_test.cpp)
    target_link_libraries(unit_rm_error_map PRIVATE realman_driver)
endif()

# ---- Modbus RTU 帧编解码单元测试（纯函数，mock 也可跑）----
if(TARGET linkerhand_codec)
    robot_add_test(unit_modbus_frame_codec unit/modbus_frame_codec_test.cpp)
    target_link_libraries(unit_modbus_frame_codec PRIVATE linkerhand_codec)
endif()

# ---- RmPassthroughModbus 传输层测试（--wrap 模拟 RM 寄存器 API）----
# 仅 debug/hardware preset（linkerhand_driver 构建）时注册。
if(ENABLE_LINKERHAND_DRIVER AND TARGET linkerhand_driver)
    add_executable(unit_rm_passthrough unit/rm_passthrough_modbus_test.cpp)
    target_link_libraries(unit_rm_passthrough PRIVATE
        robotics_core mock_devices ${GTEST_LIB} linkerhand_driver)
    target_include_directories(unit_rm_passthrough PRIVATE
        ${CMAKE_SOURCE_DIR}/src
        ${RM_API2_ROOT}/include)
    # 将 rm_* 符号重定向到测试内的 __wrap_rm_*，模拟 RM 返回值
    target_link_options(unit_rm_passthrough PRIVATE
        -Wl,--wrap=rm_read_input_registers
        -Wl,--wrap=rm_read_multiple_input_registers
        -Wl,--wrap=rm_read_holding_registers
        -Wl,--wrap=rm_read_multiple_holding_registers
        -Wl,--wrap=rm_write_single_register
        -Wl,--wrap=rm_write_registers)
    add_test(NAME unit_rm_passthrough COMMAND unit_rm_passthrough)
endif()

# ---- 集成测试（Mock 双设备）----
robot_add_test(integration_mock_double integration/mock_double_test.cpp)
robot_add_test(integration_mock_record integration/mock_record_test.cpp)

# ---- 硬件测试（需真实设备，默认不构建、注册但 DISABLED）----
if(ENABLE_HARDWARE_TESTS)
    add_executable(hw_arm_connect hardware/hw_arm_connect_test.cpp)
    target_link_libraries(hw_arm_connect PRIVATE robotics_core mock_devices ${GTEST_LIB})
    target_include_directories(hw_arm_connect PRIVATE ${CMAKE_SOURCE_DIR}/src)
    if(TARGET realman_driver)
        target_link_libraries(hw_arm_connect PRIVATE realman_driver)
    endif()
    # 注册进 ctest 但默认 DISABLED，需显式 ctest -R hw_arm_connect --no-tests=error 运行
    add_test(NAME hw_arm_connect COMMAND hw_arm_connect)
    set_tests_properties(hw_arm_connect PROPERTIES DISABLED TRUE)
endif()
