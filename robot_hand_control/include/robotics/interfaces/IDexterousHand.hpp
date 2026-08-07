#pragma once

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/states/DeviceHealth.hpp"
#include "robotics/domain/states/DexterousHandState.hpp"
#include "robotics/domain/types/JointVector.hpp"

namespace robotics::domain {

/// 灵巧手预设姿态
enum class HandPreset {
    Open,           ///< 完全张开 {255×6}
    Close,          ///< 握拳 {0×6}
    PreGrasp,       ///< 预抓取 {255,128,255,255,255,255}
    Custom,         ///< 自定义
};

/**
 * @brief 灵巧手能力抽象（不是 O6 寄存器）。
 */
class IDexterousHand {
public:
    virtual ~IDexterousHand() = default;

    // ---- 连接管理 ----
    virtual Result connect() = 0;
    virtual Result disconnect() = 0;
    virtual bool is_connected() const = 0;

    // ---- 状态 ----
    virtual DexterousHandState get_state() const = 0;

    // ---- 关节控制 ----
    /// 设置 6 通道位置（raw 0-255；或按转换语义）
    virtual Result set_joint_positions(const HandJointVector& positions) = 0;
    virtual Result set_joint_speeds(const HandJointVector& speeds) = 0;
    virtual Result set_torque_limits(const HandJointVector& torques) = 0;

    // ---- 预设 ----
    virtual Result apply_preset(HandPreset preset) = 0;

    // ---- 停止/诊断 ----
    virtual Result stop() = 0;
    virtual Result clear_error() = 0;
    virtual DeviceHealth health_check() = 0;
};

}  // namespace robotics::domain
