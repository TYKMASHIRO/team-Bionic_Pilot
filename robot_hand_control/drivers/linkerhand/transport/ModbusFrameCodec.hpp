#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace robotics::linkerhand {

/**
 * @brief Modbus RTU 帧编解码辅助（纯函数，无厂商依赖）。
 *
 * 职责：
 *   1. CRC16 (Modbus) 计算与校验
 *   2. 解析请求帧（slave_id / 功能码 / 寄存器地址 / 数量 / 数据）
 *   3. 组响应帧（读输入/保持寄存器、写单寄存器 echo、写多寄存器）
 *
 * O6 使用 Modbus RTU（数据位 8 / 停止位 1 / 校验 none / 波特率 115200）。
 * 上层（linkerhand SDK）回调要求收发完整帧；本类只做字节层面编解码。
 */
namespace modbus_frame {

/// Modbus 功能码（O6 协议支持 04 读输入、16/0x10 写保持）
enum class FunctionCode : uint8_t {
    ReadInputRegisters = 0x04,
    ReadHoldingRegisters = 0x03,
    WriteSingleRegister = 0x06,
    WriteMultipleRegisters = 0x10,
};

/// 解析后的请求帧信息
struct RequestInfo {
    uint8_t slave_id = 0;
    uint8_t function = 0;
    uint16_t address = 0;
    uint16_t count = 0;            ///< 寄存器数量（写多寄存器 = data_len/2）
    uint16_t byte_count = 0;       ///< 数据字节数（写多寄存器，frame[6]）
    bool valid = false;            ///< 帧头 + CRC 校验是否通过
};

/// 读寄存器响应信息（0x04 / 0x03）
struct ReadResponse {
    uint8_t slave_id = 0;
    uint8_t function = 0;
    uint16_t count = 0;            ///< 寄存器数量
    std::vector<uint16_t> values;  ///< 寄存器值（大端解析，避免未对齐访问）
    bool valid = false;
};

/// CRC16 (Modbus) 计算；poly 0xA001，初值 0xFFFF
uint16_t crc16(const uint8_t* data, size_t len);

/// 校验整帧 CRC（帧 = 前 len-2 字节 + 尾 2 字节 CRC，小端）
bool check_crc(const uint8_t* frame, size_t len);

/// 解析请求帧（含 CRC 校验）。len 不足或 CRC 错误 → valid=false
RequestInfo parse_request(const uint8_t* frame, size_t len);

/// 解析读寄存器响应帧（0x04/0x03，含 CRC 校验）
ReadResponse parse_read_response(const uint8_t* frame, size_t len);

/// 组读寄存器响应：slave + FC + 字节数 + 寄存器大端数据 + CRC
/// 返回总字节数；buf 容量不足返回 0。
size_t build_read_response(uint8_t* buf, size_t capacity, uint8_t slave_id,
                           uint8_t function, const uint16_t* regs,
                           size_t reg_count);

/// 组写单寄存器响应（0x06 echo：slave + FC + addr + 值 + CRC）
size_t build_write_single_response(uint8_t* buf, size_t capacity,
                                   uint8_t slave_id, uint16_t address,
                                   uint16_t value);

/// 组写多寄存器响应（0x10：slave + FC + addr + count + CRC）
size_t build_write_multiple_response(uint8_t* buf, size_t capacity,
                                     uint8_t slave_id, uint16_t address,
                                     size_t reg_count);

}  // namespace modbus_frame

}  // namespace robotics::linkerhand
