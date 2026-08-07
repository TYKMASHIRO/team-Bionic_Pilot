# 应用：robotctl（CLI）+ diagnostics

add_executable(robotctl robotctl/main.cpp)
target_link_libraries(robotctl PRIVATE robotics_core mock_devices)

add_executable(diagnostics diagnostics/main.cpp)
target_link_libraries(diagnostics PRIVATE robotics_core)

# 可选：驱动接入真实设备（阶段2/3后放开）
# HAVE_*_DRIVER 宏供 main.cpp 条件包含驱动头（mock 构建无驱动时编译不过驱动路径）
if(ENABLE_REALMAN_DRIVER AND TARGET realman_driver)
    target_link_libraries(robotctl PRIVATE realman_driver)
    target_compile_definitions(robotctl PRIVATE HAVE_REALMAN_DRIVER=1)
endif()
if(ENABLE_LINKERHAND_DRIVER AND TARGET linkerhand_driver)
    target_link_libraries(robotctl PRIVATE linkerhand_driver)
    target_compile_definitions(robotctl PRIVATE HAVE_LINKERHAND_DRIVER=1)
endif()
