// Modbus RTU 帧编解码单元测试（纯函数，无硬件）
#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "drivers/linkerhand/transport/ModbusFrameCodec.hpp"

namespace codec = robotics::linkerhand::modbus_frame;

namespace {

/// 构造含 CRC 的完整请求帧
std::vector<uint8_t> make_request(std::initializer_list<uint8_t> head) {
    std::vector<uint8_t> frame(head);
    const uint16_t crc = codec::crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));
    return frame;
}

}  // namespace

TEST(ModbusFrameCodec, Crc16KnownVector) {
    // Modbus 标准测试向量：01 03 00 00 00 0A → CRC 0xCDC5
    const uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x0A};
    EXPECT_EQ(codec::crc16(data, sizeof(data)), 0xCDC5u);
}

TEST(ModbusFrameCodec, CheckCrcValidAndCorrupt) {
    const std::vector<uint8_t> frame = make_request({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A});
    EXPECT_TRUE(codec::check_crc(frame.data(), frame.size()));

    std::vector<uint8_t> bad = frame;
    bad[2] ^= 0xFF;  // 篡改地址字段
    EXPECT_FALSE(codec::check_crc(bad.data(), bad.size()));
}

TEST(ModbusFrameCodec, ParseReadInputRequest) {
    // O6 读位置：slave 0x27, FC 0x04, addr 0, count 6
    const auto frame = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x06});
    const auto info = codec::parse_request(frame.data(), frame.size());
    ASSERT_TRUE(info.valid);
    EXPECT_EQ(info.slave_id, 0x27);
    EXPECT_EQ(info.function, 0x04);
    EXPECT_EQ(info.address, 0);
    EXPECT_EQ(info.count, 6);
}

TEST(ModbusFrameCodec, ParseWriteMultipleRequest) {
    // O6 写 6 通道位置：slave 0x27, FC 0x10, addr 0, count 6, byte_count 12
    std::vector<uint8_t> frame = {
        0x27, 0x10, 0x00, 0x00, 0x00, 0x06, 0x0C,
        0xFF, 0xFF, 0x80, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
        0xFF, 0xFF,  // 共 12 字节数据（6 寄存器 × 2 字节）
    };
    const uint16_t crc = codec::crc16(frame.data(), frame.size());
    frame.push_back(static_cast<uint8_t>(crc & 0xFF));
    frame.push_back(static_cast<uint8_t>((crc >> 8) & 0xFF));

    const auto info = codec::parse_request(frame.data(), frame.size());
    ASSERT_TRUE(info.valid);
    EXPECT_EQ(info.function, 0x10);
    EXPECT_EQ(info.count, 6);
    EXPECT_EQ(info.byte_count, 12);
}

TEST(ModbusFrameCodec, ParseInvalidCrcRejected) {
    std::vector<uint8_t> frame = make_request({0x27, 0x04, 0x00, 0x00, 0x00, 0x06});
    frame[frame.size() - 1] ^= 0x01;
    const auto info = codec::parse_request(frame.data(), frame.size());
    EXPECT_FALSE(info.valid);
}

TEST(ModbusFrameCodec, ParseTooShortRejected) {
    const uint8_t frame[] = {0x27, 0x04, 0x00};
    const auto info = codec::parse_request(frame, sizeof(frame));
    EXPECT_FALSE(info.valid);
}

TEST(ModbusFrameCodec, BuildReadResponse) {
    uint8_t buf[64];
    const uint16_t regs[2] = {0x0102, 0x0304};
    const size_t n = codec::build_read_response(buf, sizeof(buf), 0x27, 0x04, regs, 2);
    ASSERT_EQ(n, 9u);  // 27 04 04 01 02 03 04 CRC2
    EXPECT_EQ(buf[0], 0x27);
    EXPECT_EQ(buf[1], 0x04);
    EXPECT_EQ(buf[2], 0x04);          // 字节数
    EXPECT_EQ(buf[3], 0x01);          // reg[0] 高字节
    EXPECT_EQ(buf[4], 0x02);          // reg[0] 低字节
    EXPECT_EQ(buf[5], 0x03);
    EXPECT_EQ(buf[6], 0x04);
    EXPECT_TRUE(codec::check_crc(buf, n));
}

TEST(ModbusFrameCodec, BuildWriteSingleResponse) {
    uint8_t buf[64];
    const size_t n = codec::build_write_single_response(buf, sizeof(buf), 0x27, 0x0000, 0x00FF);
    ASSERT_EQ(n, 8u);
    EXPECT_EQ(buf[0], 0x27);
    EXPECT_EQ(buf[1], 0x06);
    EXPECT_EQ(buf[2], 0x00);
    EXPECT_EQ(buf[3], 0x00);
    EXPECT_EQ(buf[4], 0x00);
    EXPECT_EQ(buf[5], 0xFF);
    EXPECT_TRUE(codec::check_crc(buf, n));
}

TEST(ModbusFrameCodec, BuildWriteMultipleResponse) {
    uint8_t buf[64];
    const size_t n = codec::build_write_multiple_response(buf, sizeof(buf), 0x27, 0x0000, 6);
    ASSERT_EQ(n, 8u);
    EXPECT_EQ(buf[0], 0x27);
    EXPECT_EQ(buf[1], 0x10);
    EXPECT_EQ(buf[2], 0x00);
    EXPECT_EQ(buf[3], 0x00);
    EXPECT_EQ(buf[4], 0x00);
    EXPECT_EQ(buf[5], 0x06);
    EXPECT_TRUE(codec::check_crc(buf, n));
}

TEST(ModbusFrameCodec, ParseReadResponse) {
    uint8_t frame[64];
    const uint16_t regs[3] = {0x00FF, 0x8000, 0x1234};
    const size_t n = codec::build_read_response(frame, sizeof(frame), 0x27, 0x04, regs, 3);
    const auto resp = codec::parse_read_response(frame, n);
    ASSERT_TRUE(resp.valid);
    EXPECT_EQ(resp.slave_id, 0x27);
    EXPECT_EQ(resp.function, 0x04);
    ASSERT_EQ(resp.count, 3u);
    ASSERT_EQ(resp.values.size(), 3u);
    EXPECT_EQ(resp.values[0], 0x00FF);
    EXPECT_EQ(resp.values[1], 0x8000);
    EXPECT_EQ(resp.values[2], 0x1234);
}

TEST(ModbusFrameCodec, BufferTooSmallReturnsZero) {
    const uint16_t regs[2] = {0x0102, 0x0304};
    uint8_t tiny[4];
    EXPECT_EQ(codec::build_read_response(tiny, sizeof(tiny), 0x27, 0x04, regs, 2), 0u);
    EXPECT_EQ(codec::build_write_single_response(tiny, sizeof(tiny), 0x27, 0, 1), 0u);
}
