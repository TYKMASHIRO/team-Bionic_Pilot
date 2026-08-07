#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "robotics/domain/types/DeviceType.hpp"
#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/// 命令 ID（全局唯一）
using CommandId = std::string;

/// 命令类型
enum class CommandType {
    Unknown = 0,
    ArmMoveJoint,       ///< 机械臂关节运动
    ArmMovePose,        ///< 机械臂位姿运动
    ArmMoveLinear,      ///< 机械臂直线运动
    ArmStop,            ///< 机械臂停止
    ArmEmergencyStop,   ///< 机械臂急停
    ArmDragTeachStart,  ///< 开始拖动示教
    ArmDragTeachStop,   ///< 结束拖动示教
    ArmTrajectoryReplay,///< 轨迹复现
    HandSetPosition,    ///< 灵巧手关节位置
    HandSetSpeed,       ///< 灵巧手速度
    HandSetTorque,      ///< 灵巧手转矩
    HandApplyPreset,    ///< 灵巧手预设
    HandStop,           ///< 灵巧手停止
    SkillRun,           ///< 执行 Skill
    RecordStart,        ///< 开始记录
    RecordStop,         ///< 停止记录
};

/// 命令/任务状态机
enum class CommandState {
    Created = 0,
    Validating,
    Queued,
    Running,
    Succeeded,
    Failed,
    Cancelling,
    Cancelled,
    TimedOut,
    SafetyStopped,
};

/**
 * @brief 统一命令描述。
 * 所有设备命令（无论 arm/hand/skill）统一经 CommandScheduler 串行化。
 */
struct Command {
    CommandId command_id;
    CommandType type = CommandType::Unknown;
    DeviceType target_device = DeviceType::Unknown;

    // 参数以字符串键值承载（具体解释由 Skill/Adapter 完成）
    // 一期保持轻量；后续可替换为强类型参数。
    std::string parameters_json;

    Timestamp create_time;
    std::chrono::steady_clock::time_point deadline{};  ///< 执行截止
    CommandState state = CommandState::Created;
    bool cancelled = false;  ///< 取消令牌
};

/// 生成一个新的命令 ID（基于单调序号）
CommandId make_command_id(const char* prefix = "cmd");

std::string to_string(CommandType type);
std::string to_string(CommandState state);

}  // namespace robotics::domain
