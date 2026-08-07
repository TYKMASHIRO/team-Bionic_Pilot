#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

#include "communication/IModbus.h"

namespace robotics::linkerhand {

/**
 * @brief O6 直连串口传输层（扩展路径：电脑 USB-RS485 直连，占位）。
 *
 * 当前唯一可用路径是 RM75 末端 RS485 透传（RmPassthroughModbus）。
 * 直连实现（打开 /dev/ttyUSB0 等）尚未验证，故只建立接口与占位：
 * 所有操作返回失败/空，不伪造未验证行为。
 */
class DirectSerialModbus : public ::linkerhand::communication::IModbus {
public:
    explicit DirectSerialModbus(std::string device = "/dev/ttyUSB0",
                                int baudrate = 115200);
    ~DirectSerialModbus() override;

    DirectSerialModbus(const DirectSerialModbus&) = delete;
    DirectSerialModbus& operator=(const DirectSerialModbus&) = delete;

    bool isOpen() const override;
    void close() override;
    bool sendRawFrame(const uint8_t* data, size_t length) override;
    int receiveCompleteFrame(uint8_t* buffer, size_t max_size,
                             int timeout_ms = 500) override;
    int transact(const uint8_t* request, size_t request_len,
                 uint8_t* response, size_t max_response_len,
                 int timeout_ms = 500) override;

private:
    std::string device_;
    int baudrate_;
};

}  // namespace robotics::linkerhand
