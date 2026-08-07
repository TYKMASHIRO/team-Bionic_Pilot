# RealMan RM75 driver（阶段2实现）
# 需要 RM_API2 库。仅在 ENABLE_REALMAN_DRIVER=ON 时构建。
add_library(realman_driver
    src/RealManAdapter.cpp
)

target_include_directories(realman_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RM_API2_ROOT}/include
)

target_link_libraries(realman_driver PUBLIC robotics_core)

# 通过导入库链接 RM API（避免直接硬编码 .so 路径污染上层）
add_library(realman_api SHARED IMPORTED)
set_target_properties(realman_api PROPERTIES
    IMPORTED_LOCATION "${RM_API2_LIB_PATH}"
)
target_link_libraries(realman_driver PRIVATE realman_api)
