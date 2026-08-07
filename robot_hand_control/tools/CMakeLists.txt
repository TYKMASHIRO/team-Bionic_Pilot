# 工具：一期仅 config_validator 可运行；trajectory_inspector / recording_converter 阶段4/5实现

add_executable(config_validator config_validator/main.cpp)
target_link_libraries(config_validator PRIVATE robotics_core)

# 占位工具（阶段4/5 实现）
add_executable(trajectory_inspector trajectory_inspector/main.cpp)
target_link_libraries(trajectory_inspector PRIVATE robotics_core)

add_executable(recording_converter recording_converter/main.cpp)
target_link_libraries(recording_converter PRIVATE robotics_core)
