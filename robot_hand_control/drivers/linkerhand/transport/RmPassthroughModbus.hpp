#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>

#include "communication/IModbus.h"

namespace robotics::linkerhand {

/**
 * @brief 经 RM75 控制器 Modbus RTU 透传的 O6 传输层。
 *
 * O6 SDK 回调模式要求收发完整 Modbus RTU 帧；
 * RM75 透传是高层寄存器 API（非原始字节）。本类职责：
 *   1. 解析 O6 SDK 发来的完整 Modbus RTU 帧（slave_id/功能码/地址/数据/CRC）
 *   2. 映射到 RM 寄存器 API：
 *       0x04 → rm_read_input_registers / rm_read_multiple_input_registers
 *       0x03 → rm_read_holding_registers / rm_read_multiple_holding_registers
 *       0x06 → rm_write_single_register
 *       0x10 → rm_write_registers（O6 保持寄存器写）
 *   3. 重组 RM 返回的寄存器值为完整 Modbus RTU 响应帧（含 CRC）回给 O6 SDK
 *
 * RM 寄存器 API 限制：单读 1 个、多读 3~12 个、多写 ≤10 个。0x10 写后 RM
 * 不返回响应帧，因此回读写入值以组装标准 0x10 响应。
 */
class RmPassthroughModbus : public ::linkerhand::communication::IModbus {
public:
    /**
     * @param handle     已连接的 RM75 句柄（不持有所有权；void* 避免厂商类型入头文件）
     * @param slave_id   O6 Modbus 从站地址（右手 0x27）
     * @param timeout_ms 透传超时（毫秒）；RM 侧以百毫秒计，由上层配置
     */
    RmPassthroughModbus(void* handle, uint8_t slave_id = 0x27,
                        int timeout_ms = 500);
    ~RmPassthroughModbus() override;

    RmPassthroughModbus(const RmPassthroughModbus&) = delete;
    RmPassthroughModbus& operator=(const RmPassthroughModbus&) = delete;

    bool isOpen() const override;
    void close() override;

    bool sendRawFrame(const uint8_t* data, size_t length) override;
    int receiveCompleteFrame(uint8_t* buffer, size_t max_size,
                             int timeout_ms = 500) override;
    int transact(const uint8_t* request, size_t request_len,
                 uint8_t* response, size_t max_response_len,
                 int timeout_ms = 500) override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robotics::linkerhand
