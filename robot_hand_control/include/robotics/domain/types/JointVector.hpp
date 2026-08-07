#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace robotics::domain {

/// 机械臂自由度（RM75-6F 为 7）
inline constexpr std::size_t kArmDof = 7;

/// 灵巧手自由度（O6 为 6 通道）
inline constexpr std::size_t kHandDof = 6;

/// 机械臂关节向量（7 维，角度单位 rad）
using ArmJointVector = std::array<double, kArmDof>;

/// 灵巧手关节向量（6 维，raw 0-255 或 rad，语义由上层定义）
using HandJointVector = std::array<double, kHandDof>;

/// 通用动态关节向量（变长，用于工具/校验）
using JointVector = std::vector<double>;

/// 六维力/力矩向量：[Fx, Fy, Fz, Mx, My, Mz]（N / N·m）
using ForceTorqueVector = std::array<double, 6>;

}  // namespace robotics::domain
