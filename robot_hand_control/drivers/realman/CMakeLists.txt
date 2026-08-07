# RealMan RM75 driver（阶段2）
# 需要 RM_API2 库。仅在 ENABLE_REALMAN_DRIVER=ON 时构建。
add_library(realman_driver
    src/RealManAdapter.cpp
    src/RmErrorMap.cpp
)

target_include_directories(realman_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RM_API2_ROOT}/include
)

target_link_libraries(realman_driver PUBLIC robotics_core)

# 导入 RM API 共享库
add_library(realman_api SHARED IMPORTED)
set_target_properties(realman_api PROPERTIES
    IMPORTED_LOCATION "${RM_API2_LIB_PATH}"
)
target_link_libraries(realman_driver PRIVATE realman_api)

# 运行时 RPATH：使二进制在运行时能找到 libapi_cpp.so
set_target_properties(realman_driver PROPERTIES
    BUILD_RPATH "${RM_API2_ROOT}/linux/linux_x86_c++_v1.1.6"
)
