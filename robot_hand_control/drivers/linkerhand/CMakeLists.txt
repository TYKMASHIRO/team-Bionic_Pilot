# LinkerHand O6 driver（阶段3）
#
# 分层：
#   - linkerhand_codec：Modbus RTU 帧编解码（纯函数，无厂商依赖），始终可构建，
#     mock 模式也可单测帧解析/组帧/CRC。
#   - linkerhand_driver：O6 适配（RmPassthroughModbus + LinkerHandAdapter +
#     DirectSerialModbus 占位），仅在 ENABLE_LINKERHAND_DRIVER=ON 时构建，
#     链接 linkerhand SDK。

# ---------------------------------------------------------------------------
# 帧编解码（无厂商依赖）
# ---------------------------------------------------------------------------
add_library(linkerhand_codec STATIC
    transport/ModbusFrameCodec.cpp
)
target_include_directories(linkerhand_codec PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/transport
)
target_link_libraries(linkerhand_codec PUBLIC robotics_core)

# ---------------------------------------------------------------------------
# O6 驱动（需要 linkerhand SDK）
# ---------------------------------------------------------------------------
if(ENABLE_LINKERHAND_DRIVER)
    add_library(linkerhand_driver STATIC
        src/LinkerHandAdapter.cpp
        transport/RmPassthroughModbus.cpp
        transport/DirectSerialModbus.cpp
    )

    target_include_directories(linkerhand_driver PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${LINKERHAND_SDK_ROOT}/include
        ${LINKERHAND_SDK_ROOT}/include/api
        ${LINKERHAND_SDK_ROOT}/include/core
        ${LINKERHAND_SDK_ROOT}/include/communication
        ${RM_API2_ROOT}/include   # RmPassthroughModbus 经 RM 寄存器 API 透传
    )

    target_link_libraries(linkerhand_driver PUBLIC robotics_core linkerhand_codec)

    # O6 依赖 RM75 透传（rm_* 符号）：若 RM 驱动开启则链接其 API 库
    if(TARGET realman_api)
        target_link_libraries(linkerhand_driver PRIVATE realman_api)
    else()
        message(WARNING
            "linkerhand_driver 依赖 RM75 透传（rm_* 符号）。未启用 ENABLE_REALMAN_DRIVER，"
            "需由最终可执行文件另行提供 rm_* 符号（如测试的 --wrap 模拟）。")
    endif()

    # 导入 linkerhand SDK 库
    add_library(linkerhand_sdk SHARED IMPORTED)
    set_target_properties(linkerhand_sdk PROPERTIES
        IMPORTED_LOCATION "${LINKERHAND_LIB_PATH}"
    )
    target_link_libraries(linkerhand_driver PRIVATE linkerhand_sdk)

    # 运行时 RPATH：使二进制运行时能找到 libcanbus 等 SDK 依赖
    get_filename_component(LINKERHAND_LIB_DIR "${LINKERHAND_LIB_PATH}" DIRECTORY)
    get_filename_component(LINKERHAND_ROOT_DIR "${LINKERHAND_SDK_ROOT}" REALPATH)
    set_target_properties(linkerhand_driver PROPERTIES
        BUILD_RPATH "${LINKERHAND_LIB_DIR}"
    )
endif()
