#pragma once

#include <string>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/states/DeviceHealth.hpp"
#include "robotics/domain/states/RobotArmState.hpp"
#include "robotics/domain/types/JointVector.hpp"
#include "robotics/domain/types/Pose.hpp"
#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/**
 * @brief 机械臂能力抽象（不是 RM75 厂商 API）。
 *
 * 所有实现（RealManAdapter / MockRobotArm）都实现本接口。
 * 上层只依赖本接口，禁止把 rm_*_t 泄漏到上层。
 */
class IRobotArm {
public:
    virtual ~IRobotArm() = default;

    // ---- 连接管理 ----
    virtual Result connect() = 0;
    virtual Result disconnect() = 0;
    virtual bool is_connected() const = 0;

    // ---- 使能 ----
    virtual Result enable() = 0;
    virtual Result disable() = 0;

    // ---- 状态 ----
    virtual RobotArmState get_state() const = 0;

    // ---- 运动 ----
    virtual Result move_joint(const ArmJointVector& joint, double speed_ratio,
                              bool block = true) = 0;
    virtual Result move_pose(const Pose& pose, double speed_ratio,
                             bool block = true) = 0;
    virtual Result move_linear(const Pose& pose, double speed_ratio,
                               bool block = true) = 0;

    // ---- 停止 ----
    virtual Result stop() = 0;
    virtual Result emergency_stop() = 0;

    // ---- 拖动示教（★ 任务一核心）----
    /// 开始拖动示教；record=true 时记录轨迹
    virtual Result start_drag_teach(bool record_trajectory) = 0;
    virtual Result stop_drag_teach() = 0;

    // ---- 轨迹复现（拖动示教记录轨迹）----
    /// 运动到轨迹起点（20% 速度）
    virtual Result trajectory_to_origin(bool block = true) = 0;
    /// 开始轨迹复现
    virtual Result replay_trajectory(bool block = true) = 0;
    /// 暂停/继续/停止复现
    virtual Result pause_trajectory() = 0;
    virtual Result continue_trajectory() = 0;
    virtual Result stop_trajectory() = 0;

    // ---- 坐标系 ----
    virtual Result set_tool_frame(const std::string& name, const Pose& pose,
                                  double payload_kg) = 0;
    virtual Result set_work_frame(const std::string& name,
                                  const Pose& pose) = 0;

    // ---- 力/诊断 ----
    virtual ForceTorque get_force_torque() const = 0;
    virtual Result clear_error() = 0;
    virtual DeviceHealth health_check() = 0;
};

}  // namespace robotics::domain
