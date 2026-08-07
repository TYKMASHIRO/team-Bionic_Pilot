#include "drivers/linkerhand/transport/ModbusFrameCodec.hpp"

namespace robotics::linkerhand {
namespace modbus_frame {

namespace {

constexpr std::size_t kCrcLen = 2;

uint16_t append_crc(uint16_t crc, uint8_t byte) {
    crc ^= byte;
    for (int i = 0; i < 8; ++i) {
        if (crc & 0x0001) {
            crc = static_cast<uint16_t>((crc >> 1) ^ 0xA001);
        } else {
            crc = static_cast<uint16_t>(crc >> 1);
        }
    }
    return crc;
}

}  // namespace

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; ++i) {
        crc = append_crc(crc, data[i]);
    }
    return crc;
}

bool check_crc(const uint8_t* frame, size_t len) {
    if (frame == nullptr || len < kCrcLen + 1) {
        return false;
    }
    const uint16_t expected = crc16(frame, len - kCrcLen);
    const uint16_t actual = static_cast<uint16_t>(
        frame[len - kCrcLen] | (frame[len - kCrcLen + 1] << 8));
    return expected == actual;
}

RequestInfo parse_request(const uint8_t* frame, size_t len) {
    RequestInfo info;
    if (frame == nullptr || len < 8) {  // 最小请求帧：slave+FC+addr2+count2+CRC2
        return info;
    }
    if (!check_crc(frame, len)) {
        return info;
    }
    info.slave_id = frame[0];
    info.function = frame[1];
    info.address = static_cast<uint16_t>((frame[2] << 8) | frame[3]);
    info.count = static_cast<uint16_t>((frame[4] << 8) | frame[5]);
    // 写多寄存器（0x10）：frame[6]=字节数，数据从 frame[7] 开始
    if (info.function == static_cast<uint8_t>(FunctionCode::WriteMultipleRegisters) &&
        len >= 9) {
        info.byte_count = frame[6];
        if (len < static_cast<size_t>(7 + info.byte_count + kCrcLen)) {
            return info;  // 数据不足，仍标记 valid（上层再校验）
        }
    }
    info.valid = true;
    return info;
}

ReadResponse parse_read_response(const uint8_t* frame, size_t len) {
    ReadResponse resp;
    if (frame == nullptr || len < 5) {
        return resp;
    }
    if (!check_crc(frame, len)) {
        return resp;
    }
    resp.slave_id = frame[0];
    resp.function = frame[1];
    resp.count = frame[2] / 2;  // 字节数 / 2 = 寄存器数
    if (len < static_cast<size_t>(3 + frame[2] + kCrcLen)) {
        return resp;  // 数据长度不足
    }
    // 逐字节大端解析，避免对帧内奇地址做未对齐 uint16_t 读取
    resp.values.reserve(resp.count);
    for (uint16_t i = 0; i < resp.count; ++i) {
        const size_t off = 3 + static_cast<size_t>(i) * 2;
        resp.values.push_back(static_cast<uint16_t>((frame[off] << 8) | frame[off + 1]));
    }
    resp.valid = true;
    return resp;
}

size_t build_read_response(uint8_t* buf, size_t capacity, uint8_t slave_id,
                           uint8_t function, const uint16_t* regs,
                           size_t reg_count) {
    if (buf == nullptr || regs == nullptr || reg_count > 255 / 2) {
        return 0;
    }
    const size_t total = 3 + reg_count * 2 + kCrcLen;
    if (capacity < total) {
        return 0;
    }
    size_t i = 0;
    buf[i++] = slave_id;
    buf[i++] = function;
    buf[i++] = static_cast<uint8_t>(reg_count * 2);
    for (size_t r = 0; r < reg_count; ++r) {
        buf[i++] = static_cast<uint8_t>((regs[r] >> 8) & 0xFF);
        buf[i++] = static_cast<uint8_t>(regs[r] & 0xFF);
    }
    const uint16_t crc = crc16(buf, i);
    buf[i++] = static_cast<uint8_t>(crc & 0xFF);
    buf[i++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return i;
}

size_t build_write_single_response(uint8_t* buf, size_t capacity,
                                   uint8_t slave_id, uint16_t address,
                                   uint16_t value) {
    if (buf == nullptr || capacity < 8) {
        return 0;
    }
    size_t i = 0;
    buf[i++] = slave_id;
    buf[i++] = static_cast<uint8_t>(FunctionCode::WriteSingleRegister);
    buf[i++] = static_cast<uint8_t>((address >> 8) & 0xFF);
    buf[i++] = static_cast<uint8_t>(address & 0xFF);
    buf[i++] = static_cast<uint8_t>((value >> 8) & 0xFF);
    buf[i++] = static_cast<uint8_t>(value & 0xFF);
    const uint16_t crc = crc16(buf, i);
    buf[i++] = static_cast<uint8_t>(crc & 0xFF);
    buf[i++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return i;
}

size_t build_write_multiple_response(uint8_t* buf, size_t capacity,
                                     uint8_t slave_id, uint16_t address,
                                     size_t reg_count) {
    if (buf == nullptr || capacity < 8 || reg_count > 0xFFFF) {
        return 0;
    }
    size_t i = 0;
    buf[i++] = slave_id;
    buf[i++] = static_cast<uint8_t>(FunctionCode::WriteMultipleRegisters);
    buf[i++] = static_cast<uint8_t>((address >> 8) & 0xFF);
    buf[i++] = static_cast<uint8_t>(address & 0xFF);
    buf[i++] = static_cast<uint8_t>((reg_count >> 8) & 0xFF);
    buf[i++] = static_cast<uint8_t>(reg_count & 0xFF);
    const uint16_t crc = crc16(buf, i);
    buf[i++] = static_cast<uint8_t>(crc & 0xFF);
    buf[i++] = static_cast<uint8_t>((crc >> 8) & 0xFF);
    return i;
}

}  // namespace modbus_frame
}  // namespace robotics::linkerhand
