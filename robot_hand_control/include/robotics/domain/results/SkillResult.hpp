#pragma once

#include <chrono>
#include <string>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/errors/Error.hpp"

namespace robotics::domain {

/**
 * @brief Skill 执行结果。
 * 至少包含：成功/失败、失败阶段、统一错误、执行时间、
 * 轨迹/配置/标定版本、设备状态摘要、是否触发安全停止。
 */
struct SkillResult {
    std::string skill_id;
    std::string skill_version;
    bool success = false;
    Error error;
    std::string failed_stage;    ///< 失败阶段名
    CommandState final_state = CommandState::Failed;
    std::chrono::milliseconds duration_ms{0};

    std::string trajectory_version;
    std::string config_version;
    std::string calibration_version;

    std::string final_device_summary;  ///< 最终设备状态摘要
    bool safety_stopped = false;       ///< 是否触发安全停止

    std::string to_string() const;
};

}  // namespace robotics::domain
