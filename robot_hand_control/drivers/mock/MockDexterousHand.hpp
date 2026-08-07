#pragma once

#include <atomic>
#include <mutex>

#include "robotics/interfaces/IDexterousHand.hpp"

namespace robotics::mock {

/**
 * @brief Mock 灵巧手。
 * 模拟连接/关节位置/预设/状态/故障。
 */
class MockDexterousHand : public domain::IDexterousHand {
public:
    MockDexterousHand() = default;

    void inject_fault(bool enable) { fault_ = enable; }
    void set_sim_temperature(double t) { sim_temperature_ = t; }

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
    mutable std::mutex mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> fault_{false};
    std::atomic<double> sim_temperature_{25.0};

    domain::HandJointVector position_{};   // raw 0-255
    domain::HandJointVector speed_{};
    domain::HandJointVector torque_{};
    domain::HandState state_{domain::HandState::Idle};
    mutable std::uint64_t sequence_ = 0;
};

}  // namespace robotics::mock
