# LinkerHand O6 driver（阶段3实现）
# 需要 linkerhand SDK。仅在 ENABLE_LINKERHAND_DRIVER=ON 时构建。
add_library(linkerhand_driver
    src/LinkerHandAdapter.cpp
    transport/RmPassthroughModbus.cpp
)

target_include_directories(linkerhand_driver PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${LINKERHAND_SDK_ROOT}/include
    ${LINKERHAND_SDK_ROOT}/include/api
    ${LINKERHAND_SDK_ROOT}/include/core
    ${LINKERHAND_SDK_ROOT}/include/communication
)

target_link_libraries(linkerhand_driver PUBLIC robotics_core)

# 导入 linkerhand SDK 库
add_library(linkerhand_sdk SHARED IMPORTED)
set_target_properties(linkerhand_sdk PROPERTIES
    IMPORTED_LOCATION "${LINKERHAND_LIB_PATH}"
)
target_link_libraries(linkerhand_driver PRIVATE linkerhand_sdk)
