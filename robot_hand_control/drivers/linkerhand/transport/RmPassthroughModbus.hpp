#pragma once

#include <cstddef>
#include <cstdint>

// 引入 O6 SDK 的 IModbus 抽象（阶段3 链接 SDK 时生效）
#include "communication/IModbus.h"

namespace robotics::linkerhand {

/**
 * @brief 经 RM75 控制器 Modbus RTU 透传的 O6 传输层。
 *
 * O6 SDK 回调模式要求收发完整 Modbus RTU 帧；
 * RM75 透传是高层寄存器 API（非原始字节）。本类职责：
 *   1. 解析 O6 完整 Modbus RTU 帧（slave_id/功能码/地址/数据/CRC）
 *   2. 映射到 RM 寄存器 API（0x04 读输入 / 0x10 写保持等）
 *   3. 重组 RM 返回值为 Modbus RTU 响应帧
 * 阶段3 完整实现；当前为占位。
 */
class RmPassthroughModbus : public ::linkerhand::communication::IModbus {
public:
    RmPassthroughModbus();
    ~RmPassthroughModbus() override;

    bool isOpen() const override;
    void close() override;

    bool sendRawFrame(const uint8_t* data, size_t length) override;
    int receiveCompleteFrame(uint8_t* buffer, size_t max_size,
                             int timeout_ms = 500) override;
    int transact(const uint8_t* request, size_t request_len,
                 uint8_t* response, size_t max_response_len,
                 int timeout_ms = 500) override;
};

}  // namespace robotics::linkerhand
