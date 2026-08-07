// DirectSerialModbus —— 占位实现（扩展路径：电脑 USB-RS485 直连 O6）。
// 未验证串口打开/帧收发，故所有操作返回失败，不伪造未验证 API。
// 将来实现：open /dev/ttyUSB* → termios 配置（8N1/115200）→ 帧收发。
#include "drivers/linkerhand/transport/DirectSerialModbus.hpp"

namespace robotics::linkerhand {

DirectSerialModbus::DirectSerialModbus(std::string device, int baudrate)
    : device_(std::move(device)), baudrate_(baudrate) {}

DirectSerialModbus::~DirectSerialModbus() = default;

bool DirectSerialModbus::isOpen() const { return false; }

void DirectSerialModbus::close() {}

bool DirectSerialModbus::sendRawFrame(const uint8_t*, size_t) { return false; }

int DirectSerialModbus::receiveCompleteFrame(uint8_t*, size_t, int) { return -1; }

int DirectSerialModbus::transact(const uint8_t*, size_t, uint8_t*, size_t, int) {
    return -1;
}

}  // namespace robotics::linkerhand
