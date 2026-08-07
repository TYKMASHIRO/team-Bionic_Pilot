#pragma once

#include "robotics/interfaces/IDexterousHand.hpp"

namespace robotics::linkerhand {

/**
 * @brief O6 灵巧手适配器（阶段3完整实现）。
 * 当前为骨架：返回 Unsupported，保证工程可编译。
 */
class LinkerHandAdapter : public domain::IDexterousHand {
public:
    LinkerHandAdapter();
    ~LinkerHandAdapter() override;

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
    domain::Result unsupported() const;
    // 阶段3: std::unique_ptr<...LinkerHandApi> api_; 等（仅存在于 .cpp）
};

}  // namespace robotics::linkerhand
