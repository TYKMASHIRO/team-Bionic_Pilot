// RmPassthroughModbus —— 阶段3 完整实现。
// 当前为骨架占位，保证工程可编译。见 docs/vendor_api_mapping.md 第1.6节。
#include "drivers/linkerhand/transport/RmPassthroughModbus.hpp"

namespace robotics::linkerhand {

RmPassthroughModbus::RmPassthroughModbus() = default;
RmPassthroughModbus::~RmPassthroughModbus() = default;

bool RmPassthroughModbus::isOpen() const {
    // 阶段3: 通过 RM 句柄判断
    return false;
}

void RmPassthroughModbus::close() {}

bool RmPassthroughModbus::sendRawFrame(const uint8_t*, size_t) {
    // 阶段3: 解析 Modbus 帧 → 映射 RM 寄存器 API
    return false;
}

int RmPassthroughModbus::receiveCompleteFrame(uint8_t*, size_t, int) {
    // 阶段3: 重组 RM 返回值为 Modbus 响应帧
    return -1;
}

int RmPassthroughModbus::transact(const uint8_t*, size_t, uint8_t*, size_t,
                                  int) {
    return -1;
}

}  // namespace robotics::linkerhand
