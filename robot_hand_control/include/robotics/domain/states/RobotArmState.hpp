#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "robotics/domain/types/DeviceType.hpp"
#include "robotics/domain/types/ForceTorque.hpp"
#include "robotics/domain/types/JointVector.hpp"
#include "robotics/domain/types/Pose.hpp"
#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/**
 * @brief 机械臂关节级状态（7 关节）。
 * 角度单位 rad；速度 rad/s；电流 A；温度 ℃。
 */
struct RobotArmJointState {
    std::array<double, kArmDof> position{};
    std::array<double, kArmDof> velocity{};
    std::array<double, kArmDof> current{};
    std::array<double, kArmDof> temperature{};
    std::array<bool, kArmDof> enabled{};
    std::array<std::uint16_t, kArmDof> error_code{};
    bool valid = false;
    Timestamp timestamp;
    std::uint64_t sequence = 0;

    ArmJointVector position_vector() const { return position; }
};

/**
 * @brief 机械臂完整状态快照（不可变）。
 */
struct RobotArmState {
    RobotArmJointState joint_state;
    Pose tcp_pose;               ///< TCP 位姿（m + rad）
    ForceTorque force_torque;    ///< 六维力/力矩
    ArmMotionState motion_state = ArmMotionState::Unknown;
    bool reached_target = false; ///< 是否到位
    std::vector<int> system_errors;  ///< 系统错误码
    bool valid = false;          ///< 状态有效
    bool fresh = false;          ///< 数据新鲜
    Timestamp timestamp;
    std::uint64_t sequence = 0;
};

}  // namespace robotics::domain
