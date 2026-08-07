cmake_minimum_required(VERSION 3.22)

project(robot_hand_control
    VERSION 0.1.0
    DESCRIPTION "RM75-6F robot arm + LinkerHand O6 dexterous hand control software"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---------------------------------------------------------------------------
# 可选组件开关
# ---------------------------------------------------------------------------
option(BUILD_TESTING "Build unit/integration tests" ON)
option(ENABLE_REALMAN_DRIVER "Build RealMan RM75 driver (requires RM API2)" OFF)
option(ENABLE_LINKERHAND_DRIVER "Build LinkerHand O6 driver (requires O6 SDK)" OFF)
option(ENABLE_HARDWARE_TESTS "Build hardware tests (require real devices, default OFF)" OFF)
option(ENABLE_SANITIZERS "Build with Address/UB sanitizers" OFF)
option(BUILD_TOOLS "Build auxiliary tools" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" OFF)

# ---------------------------------------------------------------------------
# 编译选项
# ---------------------------------------------------------------------------
include(cmake/CompilerOptions.cmake)
include(cmake/Sanitizers.cmake)
include(cmake/Dependencies.cmake)
include(cmake/Packaging.cmake)

# ---------------------------------------------------------------------------
# 厂商 SDK 根路径（供 drivers 子目录使用）
# ---------------------------------------------------------------------------
get_filename_component(TEAM_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)
set(RM_API2_ROOT "${TEAM_ROOT}/RM_API2/C++" CACHE PATH "Path to RM API2 C++ SDK")
set(LINKERHAND_SDK_ROOT "${TEAM_ROOT}/linkerhand-cpp-sdk" CACHE PATH "Path to LinkerHand SDK")
set(RM_API2_LIB_PATH "${RM_API2_ROOT}/linux/linux_x86_c++_v1.1.6/libapi_cpp.so" CACHE FILEPATH "Path to RM API2 shared library")
set(LINKERHAND_LIB_PATH "${LINKERHAND_SDK_ROOT}/lib/linux/x86_64/linkerhand_cpp_sdk.so.2.0.0" CACHE FILEPATH "Path to LinkerHand SDK library")

# ---------------------------------------------------------------------------
# 核心库（领域类型 + 接口 + 服务实现，不依赖任何厂商 SDK）
# ---------------------------------------------------------------------------
add_library(robotics_core
    src/domain/errors/Error.cpp
    src/domain/commands/Command.cpp
    src/domain/types/DeviceType.cpp
    src/domain/states/RobotArmState.cpp
    src/domain/states/DexterousHandState.cpp
    src/domain/states/CombinedRobotState.cpp
    src/domain/states/DeviceHealth.cpp
    src/domain/results/CommandResult.cpp
    src/domain/results/SkillResult.cpp
    src/infrastructure/time/SystemClock.cpp
    src/infrastructure/recording/CsvRecordSink.cpp
    src/infrastructure/config/Configuration.cpp
    src/infrastructure/logging/Logger.cpp
    src/services/CommandScheduler.cpp
    src/services/ResourceManager.cpp
    src/services/StateStore.cpp
    src/services/StateCollector.cpp
    src/services/Recorder.cpp
    src/safety/SafetySupervisor.cpp
    src/trajectory/TrajectoryRepository.cpp
    src/orchestration/ApplicationService.cpp
)

target_include_directories(robotics_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(robotics_core PUBLIC
    Threads::Threads
    ${YAML_CPP_TARGET}
)

# ---------------------------------------------------------------------------
# Mock 设备（始终构建：无硬件测试基础）
# Mock 设备实现 robotics_core 中的接口，因此它链接 robotics_core（单向）。
# ---------------------------------------------------------------------------
add_subdirectory(drivers/mock)

# ---------------------------------------------------------------------------
# 厂商驱动（按开关构建）
# ---------------------------------------------------------------------------
if(ENABLE_REALMAN_DRIVER)
    add_subdirectory(drivers/realman)
endif()

# linkerhand 目录内：ModbusFrameCodec（纯函数，无厂商依赖）始终可构建；
# linkerhand_driver（O6 适配，需 SDK）受 ENABLE_LINKERHAND_DRIVER 门控。
add_subdirectory(drivers/linkerhand)

# ---------------------------------------------------------------------------
# 应用
# ---------------------------------------------------------------------------
add_subdirectory(apps)

# ---------------------------------------------------------------------------
# 测试
# ---------------------------------------------------------------------------
if(BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------------------
# 工具
# ---------------------------------------------------------------------------
if(BUILD_TOOLS)
    add_subdirectory(tools)
endif()
