#pragma once

#include <atomic>
#include <chrono>

#include "robotics/infrastructure/config/Configuration.hpp"
#include "robotics/interfaces/ISafetySupervisor.hpp"

namespace robotics::domain {

/**
 * @brief 安全监督器实现。
 * 独立于具体 Skill；集中检查：
 *   - 连接状态、状态新鲜度
 *   - 关节软限位、TCP 速度、六维力
 *   - O6 位置/温度
 *   - 真实运动权限、通信超时
 */
class SafetySupervisor : public ISafetySupervisor {
public:
    explicit SafetySupervisor(const robotics::infra::SafetyConfig& config);

    bool motion_allowed() const override;
    Error evaluate(const CombinedRobotState& state) const override;
    bool real_motion_enabled() const override;
    void request_stop(StopLevel level) override;

    /// 显式启用/禁用真实运动权限
    void set_real_motion_enabled(bool enabled);

    /// 停止级别
    StopLevel current_stop_level() const;

private:
    robotics::infra::SafetyConfig config_;
    std::atomic<bool> real_motion_enabled_{false};
    std::atomic<StopLevel> stop_level_{StopLevel::NormalCancel};

    // 新鲜度阈值
    bool is_fresh(const Timestamp& ts) const;
};

}  // namespace robotics::domain
