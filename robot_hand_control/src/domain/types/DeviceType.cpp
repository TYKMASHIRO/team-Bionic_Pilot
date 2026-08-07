#include "robotics/domain/types/DeviceType.hpp"

namespace robotics::domain {

std::string to_string(DeviceType type) {
    switch (type) {
        case DeviceType::Unknown: return "unknown";
        case DeviceType::RobotArm: return "robot_arm";
        case DeviceType::DexterousHand: return "dexterous_hand";
        case DeviceType::Combined: return "combined";
    }
    return "unknown";
}

std::string to_string(DeviceConnectionState state) {
    switch (state) {
        case DeviceConnectionState::Disconnected: return "disconnected";
        case DeviceConnectionState::Connecting: return "connecting";
        case DeviceConnectionState::Connected: return "connected";
        case DeviceConnectionState::Error: return "error";
    }
    return "unknown";
}

std::string to_string(ArmMotionState state) {
    switch (state) {
        case ArmMotionState::Unknown: return "unknown";
        case ArmMotionState::Idle: return "idle";
        case ArmMotionState::Moving: return "moving";
        case ArmMotionState::Paused: return "paused";
        case ArmMotionState::Reaching: return "reaching";
        case ArmMotionState::Stopped: return "stopped";
        case ArmMotionState::EmergencyStopped: return "emergency_stopped";
        case ArmMotionState::DragTeaching: return "drag_teaching";
        case ArmMotionState::Replaying: return "replaying";
    }
    return "unknown";
}

std::string to_string(HandState state) {
    switch (state) {
        case HandState::Unknown: return "unknown";
        case HandState::Idle: return "idle";
        case HandState::Moving: return "moving";
        case HandState::Stalled: return "stalled";
        case HandState::Faulted: return "faulted";
    }
    return "unknown";
}

}  // namespace robotics::domain
