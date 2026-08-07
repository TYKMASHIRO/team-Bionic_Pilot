#pragma once

#include <chrono>
#include <cstdint>
#include <string>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/states/DexterousHandState.hpp"
#include "robotics/domain/states/RobotArmState.hpp"

namespace robotics::domain {

/// 双设备同步质量
enum class SyncQuality {
    Unknown = 0,
    Good,     ///< 时间差在阈值内
    Skewed,   ///< 存在可接受偏差
    Lost,     ///< 同步丢失
};

/**
 * @brief 组合状态：RM75 + O6 统一快照。
 * 用于 Recorder 同步记录与上层协同判断。
 */
struct CombinedRobotState {
    RobotArmState arm;
    DexterousHandState hand;

    /// 机械臂与灵巧手时间差（hand - arm，单位 ns）
    std::int64_t time_delta_ns = 0;
    SyncQuality sync_quality = SyncQuality::Unknown;

    std::string current_skill;    ///< 当前 Skill id
    CommandId current_command_id; ///< 当前命令 ID

    Timestamp timestamp;
    std::uint64_t sequence = 0;

    bool arm_present() const { return arm.valid; }
    bool hand_present() const { return hand.valid; }
};

}  // namespace robotics::domain
