#pragma once

#include <cstdint>
#include <memory>

#include "robotics/interfaces/IDexterousHand.hpp"

namespace robotics::linkerhand {

/**
 * @brief O6 灵巧手适配器（阶段3 完整实现）。
 *
 * 包 LinkerHandApi(O6, RIGHT, MODBUS)，经 RM75 末端 RS485 Modbus RTU 透传
 * （RmPassthroughModbus）访问 O6。连接顺序：
 *   1. 由上层先连接 RM75（传入 rm_robot_handle*）
 *   2. connect() 配置末端 RS485 为 Modbus RTU 主站
 *   3. 构造 LinkerHandApi 并注入 Modbus Tx/Rx 回调
 *
 * 厂商类型（LinkerHandApi / rm_robot_handle）全部经 PIMPL 隔离在 src/ 内。
 */
class LinkerHandAdapter : public domain::IDexterousHand {
public:
    /**
     * @param rm_handle 已连接的 RM75 句柄（void* 避免厂商类型入头文件；不持有所有权）
     * @param slave_id  O6 Modbus 从站地址（右手 0x27）
     * @param timeout_ms 透传超时（毫秒）
     */
    LinkerHandAdapter(void* rm_handle, std::uint8_t slave_id = 0x27,
                      int timeout_ms = 500);
    ~LinkerHandAdapter() override;

    LinkerHandAdapter(const LinkerHandAdapter&) = delete;
    LinkerHandAdapter& operator=(const LinkerHandAdapter&) = delete;

    domain::Result connect() override;
    domain::Result disconnect() override;
    bool is_connected() const override;
    domain::DexterousHandState get_state() const override;

    domain::Result set_joint_positions(
        const domain::HandJointVector& positions) override;
    domain::Result set_joint_speeds(
        const domain::HandJointVector& speeds) override;
    domain::Result set_torque_limits(
        const domain::HandJointVector& torques) override;
    domain::Result apply_preset(domain::HandPreset preset) override;
    domain::Result stop() override;
    domain::Result clear_error() override;
    domain::DeviceHealth health_check() override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace robotics::linkerhand
