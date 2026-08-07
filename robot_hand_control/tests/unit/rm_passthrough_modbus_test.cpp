// RmPassthroughModbus 传输层单元测试。
// 通过链接器 --wrap 将 rm_* 符号替换为测试内的 __wrap_rm_*，模拟 RM 返回值，
// 验证"请求帧 → RM 寄存器 API 映射 → 响应帧重组"全程。无需真实硬件。
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include <rm_define.h>

#include "drivers/linkerhand/transport/ModbusFrameCodec.hpp"
#include "drivers/linkerhand/transport/RmPassthroughModbus.hpp"

namespace codec = robotics::linkerhand::modbus_frame;
using robotics::linkerhand::RmPassthroughModbus;

// ---------------------------------------------------------------------------
// 模拟 RM75 寄存器 API（被链接器 --wrap 替换）
// ---------------------------------------------------------------------------
namespace {

struct FakeRm {
    int read_rc = 0;                  // 读返回码
    int write_rc = 0;                 // 写返回码
    int single_read_value = 0;        // 单读返回值
    std::vector<int> read_values;     // 多读返回值
    std::vector<int> written_values;  // 多写记录
    int written_single = 0;           // 单写记录
    rm_peripheral_read_write_params_t last_read{};
    rm_peripheral_read_write_params_t last_write{};
};

FakeRm g_fake;

/// 模拟 RM 多读契约：写入 2*params.num 个 int8（每寄存器 2 字节）。
/// 若生产代码缓冲区少分配（count 而非 2*count），此处会越界写并触发 ASan。
void copy_read_values(int* data, const std::vector<int>& src, int num) {
    for (int i = 0; i < num * 2; ++i) {
        data[i] = (static_cast<size_t>(i) < src.size()) ? src[i] : 0;
    }
}

}  // namespace

extern "C" {

int __wrap_rm_read_input_registers(rm_robot_handle* h,
                                   rm_peripheral_read_write_params_t params,
                                   int* data) {
    (void)h;
    g_fake.last_read = params;
    if (g_fake.read_rc != 0) return g_fake.read_rc;
    data[0] = g_fake.single_read_value;
    return 0;
}

int __wrap_rm_read_multiple_input_registers(rm_robot_handle* h,
                                            rm_peripheral_read_write_params_t params,
                                            int* data) {
    (void)h;
    g_fake.last_read = params;
    if (g_fake.read_rc != 0) return g_fake.read_rc;
    copy_read_values(data, g_fake.read_values, params.num);
    return 0;
}

int __wrap_rm_read_holding_registers(rm_robot_handle* h,
                                     rm_peripheral_read_write_params_t params,
                                     int* data) {
    (void)h;
    g_fake.last_read = params;
    if (g_fake.read_rc != 0) return g_fake.read_rc;
    data[0] = g_fake.single_read_value;
    return 0;
}

int __wrap_rm_read_multiple_holding_registers(rm_robot_handle* h,
                                              rm_peripheral_read_write_params_t params,
                                              int* data) {
    (void)h;
    g_fake.last_read = params;
    if (g_fake.read_rc != 0) return g_fake.read_rc;
    copy_read_values(data, g_fake.read_values, params.num);
    return 0;
}

int __wrap_rm_write_single_register(rm_robot_handle* h,
                                    rm_peripheral_read_write_params_t params,
                                    int data) {
    (void)h;
    g_fake.last_write = params;
    if (g_fake.write_rc != 0) return g_fake.write_rc;
    g_fake.written_single = data;
    return 0;
}

int __wrap_rm_write_registers(rm_robot_handle* h,
                              rm_peripheral_read_write_params_t params,
                              int* data) {
    (void)h;
    g_fake.last_write = params;
    if (g_fake.write_rc != 0) return g_fake.write_rc;
    // 与读契约一致：写 2*num 个 int8（每寄存器 2 字节）
    g_fake.written_values.assign(data, data + params.num * 2);
    return 0;
}

}  // extern "C"

// ---------------------------------------------------------------------------
// 测试
// ---------------------------------------------------------------------------
namespace {

/// 构造含 CRC 的完整请求帧
std::vector<uint8_t> make_request(std::initializer_list<uint8_t> head) {
    std::vector<uint8_t> frame(head);
    const uint16_t crc = codec::crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

/// 构造含 CRC 的完整请求帧（vector 版，写多寄存器等变长数据用）
std::vector<uint8_t> make_request_all(std::vector<uint8_t> frame) {
    const uint16_t crc = codec::crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

void reset_fake() {
    g_fake = FakeRm{};
}

}  // namespace

// ---- 0x04 读输入寄存器（多读 6 通道位置）----
TEST(RmPassthrough, ReadInputRegistersSix) {
    reset_fake();
    // O6 位置读取：slave 0x27, FC 0x04, addr 0, count 6
    const auto req = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x06});
    // RM 多读返回原始字节（int8）：每寄存器 2 字节 = 值 {255,128,200,10,5,1} 的高/低字节
    g_fake.read_values = {0x00, 0xFF, 0x00, 0x80, 0x00, 0xC8,
                          0x00, 0x0A, 0x00, 0x05, 0x00, 0x01};
    g_fake.read_rc = 0;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    ASSERT_TRUE(mb.sendRawFrame(req.data(), req.size()));

    uint8_t resp[64];
    const int n = mb.receiveCompleteFrame(resp, sizeof(resp), 500);
    ASSERT_GT(n, 0);
    ASSERT_TRUE(codec::check_crc(resp, static_cast<size_t>(n)));

    // 响应帧：27 04 0C + 6 寄存器大端 + CRC
    EXPECT_EQ(resp[0], 0x27);
    EXPECT_EQ(resp[1], 0x04);
    EXPECT_EQ(resp[2], 12);           // 6 regs × 2 bytes
    const auto info = codec::parse_read_response(resp, static_cast<size_t>(n));
    ASSERT_TRUE(info.valid);
    ASSERT_EQ(info.values.size(), 6u);
    EXPECT_EQ(info.values[0], 255);
    EXPECT_EQ(info.values[1], 128);
    EXPECT_EQ(info.values[5], 1);

    // RM 侧参数：port=1 末端 RS485, device=0x27
    EXPECT_EQ(g_fake.last_read.port, 1);
    EXPECT_EQ(g_fake.last_read.device, 0x27);
    EXPECT_EQ(g_fake.last_read.address, 0);
    EXPECT_EQ(g_fake.last_read.num, 6);
}

// ---- 0x04 读输入寄存器（单读 1 个）----
TEST(RmPassthrough, ReadSingleInputRegister) {
    reset_fake();
    const auto req = make_request({0x27, 0x04, 0x00, 0x1F, 0x00, 0x01});  // addr 31
    g_fake.single_read_value = 0x42;
    g_fake.read_rc = 0;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    ASSERT_TRUE(mb.sendRawFrame(req.data(), req.size()));
    uint8_t resp[64];
    const int n = mb.receiveCompleteFrame(resp, sizeof(resp), 500);
    ASSERT_GT(n, 0);
    EXPECT_EQ(resp[1], 0x04);
    EXPECT_EQ(resp[3], 0x00);  // 值高字节
    EXPECT_EQ(resp[4], 0x42);  // 值低字节
    EXPECT_EQ(g_fake.last_read.address, 0x1F);
    EXPECT_EQ(g_fake.last_read.num, 1);
}

// ---- 0x10 写多寄存器（写 6 通道位置目标）----
TEST(RmPassthrough, WriteMultipleRegistersSix) {
    reset_fake();
    // slave 0x27, FC 0x10, addr 0, count 6, byte_count 12, 数据 {0..5}
    std::vector<uint8_t> head = {0x27, 0x10, 0x00, 0x00, 0x00, 0x06, 0x0C};
    for (int i = 0; i < 6; ++i) {
        head.push_back(0x00);
        head.push_back(static_cast<uint8_t>(i));
    }
    const auto req = make_request_all(head);
    g_fake.write_rc = 0;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    ASSERT_TRUE(mb.sendRawFrame(req.data(), req.size()));

    uint8_t resp[64];
    const int n = mb.receiveCompleteFrame(resp, sizeof(resp), 500);
    ASSERT_GT(n, 0);
    ASSERT_TRUE(codec::check_crc(resp, static_cast<size_t>(n)));
    // 0x10 响应：27 10 00 00 00 06 + CRC
    EXPECT_EQ(resp[0], 0x27);
    EXPECT_EQ(resp[1], 0x10);
    EXPECT_EQ(resp[4], 0x00);
    EXPECT_EQ(resp[5], 0x06);

    // RM 侧写入参数与数据（2*num 字节：每寄存器高字节 0x00 + 低字节值）
    EXPECT_EQ(g_fake.last_write.port, 1);
    EXPECT_EQ(g_fake.last_write.device, 0x27);
    EXPECT_EQ(g_fake.last_write.address, 0);
    EXPECT_EQ(g_fake.last_write.num, 6);
    ASSERT_EQ(g_fake.written_values.size(), 12u);
    EXPECT_EQ(g_fake.written_values[0], 0x00);   // reg0 高字节
    EXPECT_EQ(g_fake.written_values[1], 0x00);   // reg0 低字节
    EXPECT_EQ(g_fake.written_values[3], 0x01);   // reg1 低字节
    EXPECT_EQ(g_fake.written_values[11], 0x05);  // reg5 低字节
}

// ---- 0x06 写单寄存器 ----
TEST(RmPassthrough, WriteSingleRegister) {
    reset_fake();
    const auto req = make_request({0x27, 0x06, 0x00, 0x00, 0x00, 0x80});
    g_fake.write_rc = 0;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    ASSERT_TRUE(mb.sendRawFrame(req.data(), req.size()));
    uint8_t resp[64];
    const int n = mb.receiveCompleteFrame(resp, sizeof(resp), 500);
    ASSERT_GT(n, 0);
    EXPECT_EQ(resp[0], 0x27);
    EXPECT_EQ(resp[1], 0x06);
    EXPECT_EQ(resp[5], 0x80);
    EXPECT_EQ(g_fake.written_single, 0x80);
}

// ---- 错误路径：RM 读失败 → sendRawFrame 失败 ----
TEST(RmPassthrough, ReadFailsReturnsFalse) {
    reset_fake();
    const auto req = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x06});
    g_fake.read_rc = -2;  // RM 超时

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    EXPECT_FALSE(mb.sendRawFrame(req.data(), req.size()));
}

// ---- 错误路径：RM 写失败 ----
TEST(RmPassthrough, WriteFailsReturnsFalse) {
    reset_fake();
    const auto req = make_request({0x27, 0x10, 0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x01});
    g_fake.write_rc = 1;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    EXPECT_FALSE(mb.sendRawFrame(req.data(), req.size()));
}

// ---- 错误路径：从站地址不匹配 → 拒绝 ----
TEST(RmPassthrough, WrongSlaveIdRejected) {
    reset_fake();
    const auto req = make_request({0x28, 0x04, 0x00, 0x00, 0x00, 0x06});  // 左手 0x28
    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    EXPECT_FALSE(mb.sendRawFrame(req.data(), req.size()));
}

// ---- 错误路径：不支持的寄存器数量（RM 多读限制 3..12）----
TEST(RmPassthrough, MultiReadCountTwoRejected) {
    reset_fake();
    const auto req = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x02});
    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    EXPECT_FALSE(mb.sendRawFrame(req.data(), req.size()));
}

// ---- 错误路径：无效 CRC ----
TEST(RmPassthrough, InvalidCrcRejected) {
    reset_fake();
    auto req = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x06});
    req[req.size() - 1] ^= 0xFF;
    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    EXPECT_FALSE(mb.sendRawFrame(req.data(), req.size()));
}

// ---- transact 独立路径（不复用缓存）----
TEST(RmPassthrough, TransactDirect) {
    reset_fake();
    const auto req = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x01});
    g_fake.single_read_value = 7;
    g_fake.read_rc = 0;

    RmPassthroughModbus mb(reinterpret_cast<void*>(0x1), 0x27, 500);
    uint8_t resp[64];
    const int n = mb.transact(req.data(), req.size(), resp, sizeof(resp), 500);
    ASSERT_GT(n, 0);
    EXPECT_EQ(resp[3], 0x00);
    EXPECT_EQ(resp[4], 0x07);
}
