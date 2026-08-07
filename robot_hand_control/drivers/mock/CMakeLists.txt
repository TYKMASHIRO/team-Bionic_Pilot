# Mock 设备库：始终构建，无硬件依赖
add_library(mock_devices
    MockRobotArm.cpp
    MockDexterousHand.cpp
)

target_include_directories(mock_devices PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_link_libraries(mock_devices PUBLIC robotics_core)
