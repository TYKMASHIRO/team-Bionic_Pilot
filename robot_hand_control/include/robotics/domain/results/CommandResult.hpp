#pragma once

#include <chrono>
#include <string>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/errors/Error.hpp"

namespace robotics::domain {

/**
 * @brief 命令执行结果。
 */
struct CommandResult {
    CommandId command_id;
    CommandType type = CommandType::Unknown;
    bool success = false;
    Error error;
    CommandState final_state = CommandState::Failed;
    std::chrono::milliseconds duration_ms{0};  ///< 实际执行耗时
    std::string summary;                       ///< 人类可读摘要

    std::string to_string() const;
};

}  // namespace robotics::domain
