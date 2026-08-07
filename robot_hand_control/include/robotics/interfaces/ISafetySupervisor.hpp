#pragma once

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/states/CombinedRobotState.hpp"

namespace robotics::domain {

/// 停止级别
enum class StopLevel {
    NormalCancel = 0,   ///< 受控结束当前动作
    ControlledStop,     ///< 尽快停止，保持可恢复
    EmergencyStop,      ///< 急停，需人工复位
};

/**
 * @brief 安全监督器。
 * 独立于具体 Skill，集中检查新鲜度/限位/力/温度/通信/权限。
 */
class ISafetySupervisor {
public:
    virtual ~ISafetySupervisor() = default;

    /// 评估当前组合状态，返回是否允许运动
    virtual bool motion_allowed() const = 0;

    /// 评估当前组合状态，返回第一个安全违规错误（无则返回 ok）
    virtual Error evaluate(const CombinedRobotState& state) const = 0;

    /// 是否允许真实运动（由显式权限决定）
    virtual bool real_motion_enabled() const = 0;

    /// 显式启用/禁用真实运动权限（--enable-motion）
    virtual void set_real_motion_enabled(bool enabled) = 0;

    /// 请求停止
    virtual void request_stop(StopLevel level) = 0;
};

}  // namespace robotics::domain
