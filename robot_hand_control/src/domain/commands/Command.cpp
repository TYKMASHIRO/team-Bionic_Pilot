#include "robotics/domain/commands/Command.hpp"

#include <atomic>

namespace robotics::domain {

namespace {
std::atomic<std::uint64_t> g_cmd_seq{0};
}  // namespace

CommandId make_command_id(const char* prefix) {
    const auto n = g_cmd_seq.fetch_add(1);
    return std::string(prefix) + "_" + std::to_string(n);
}

std::string to_string(CommandType type) {
    switch (type) {
        case CommandType::Unknown: return "unknown";
        case CommandType::ArmMoveJoint: return "arm_move_joint";
        case CommandType::ArmMovePose: return "arm_move_pose";
        case CommandType::ArmMoveLinear: return "arm_move_linear";
        case CommandType::ArmStop: return "arm_stop";
        case CommandType::ArmEmergencyStop: return "arm_emergency_stop";
        case CommandType::ArmDragTeachStart: return "arm_drag_teach_start";
        case CommandType::ArmDragTeachStop: return "arm_drag_teach_stop";
        case CommandType::ArmTrajectoryReplay: return "arm_trajectory_replay";
        case CommandType::HandSetPosition: return "hand_set_position";
        case CommandType::HandSetSpeed: return "hand_set_speed";
        case CommandType::HandSetTorque: return "hand_set_torque";
        case CommandType::HandApplyPreset: return "hand_apply_preset";
        case CommandType::HandStop: return "hand_stop";
        case CommandType::SkillRun: return "skill_run";
        case CommandType::RecordStart: return "record_start";
        case CommandType::RecordStop: return "record_stop";
    }
    return "unknown";
}

std::string to_string(CommandState state) {
    switch (state) {
        case CommandState::Created: return "created";
        case CommandState::Validating: return "validating";
        case CommandState::Queued: return "queued";
        case CommandState::Running: return "running";
        case CommandState::Succeeded: return "succeeded";
        case CommandState::Failed: return "failed";
        case CommandState::Cancelling: return "cancelling";
        case CommandState::Cancelled: return "cancelled";
        case CommandState::TimedOut: return "timed_out";
        case CommandState::SafetyStopped: return "safety_stopped";
    }
    return "unknown";
}

}  // namespace robotics::domain
