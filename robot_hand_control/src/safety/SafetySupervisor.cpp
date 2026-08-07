#include "robotics/safety/SafetySupervisor.hpp"

#include <algorithm>
#include <cmath>

namespace robotics::domain {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
}

SafetySupervisor::SafetySupervisor(const robotics::infra::SafetyConfig& config)
    : config_(config) {}

void SafetySupervisor::set_real_motion_enabled(bool enabled) {
    real_motion_enabled_ = enabled;
}

bool SafetySupervisor::real_motion_enabled() const {
    return real_motion_enabled_;
}

bool SafetySupervisor::is_fresh(const Timestamp& ts) const {
    const auto now = std::chrono::steady_clock::now();
    const auto age = std::chrono::duration_cast<std::chrono::milliseconds>(now - ts.steady);
    return age.count() <= config_.max_state_delay_ms;
}

bool SafetySupervisor::motion_allowed() const {
    if (!real_motion_enabled_) return false;
    return stop_level_ == StopLevel::NormalCancel;
}

Error SafetySupervisor::evaluate(const CombinedRobotState& state) const {
    // 1. 真实运动权限
    if (!real_motion_enabled_) {
        return Error::make(ErrorCategory::Safety, DeviceType::Combined,
                           "SafetySupervisor", 1,
                           "真实运动未启用（需 --enable-motion）",
                           Severity::Error, false);
    }

    // 2. 状态新鲜度
    if (state.arm.valid && !is_fresh(state.arm.timestamp)) {
        return Error::make(ErrorCategory::StateStale, DeviceType::RobotArm,
                           "SafetySupervisor", 2, "机械臂状态数据过期");
    }
    if (state.hand.valid && !is_fresh(state.hand.timestamp)) {
        return Error::make(ErrorCategory::StateStale, DeviceType::DexterousHand,
                           "SafetySupervisor", 3, "灵巧手状态数据过期");
    }

    // 3. 关节速度限制
    if (state.arm.valid) {
        const auto& vel = state.arm.joint_state.velocity;
        const double max_v = *std::max_element(vel.begin(), vel.end());
        if (std::abs(max_v) > config_.max_joint_velocity_rad_s) {
            return Error::make(ErrorCategory::Safety, DeviceType::RobotArm,
                               "SafetySupervisor", 4,
                               "关节速度超限: " + std::to_string(max_v * kRad2Deg) +
                                   " deg/s > " +
                                   std::to_string(config_.max_joint_velocity_rad_s * kRad2Deg));
        }
    }

    // 4. 六维力限制
    if (state.arm.valid) {
        const auto f = state.arm.force_torque.to_vector();
        for (int i = 0; i < 3; ++i) {
            if (std::abs(f[i]) > config_.max_force_n) {
                return Error::make(ErrorCategory::Safety, DeviceType::RobotArm,
                                   "SafetySupervisor", 5,
                                   "力超限: F[" + std::to_string(i) + "]=" +
                                       std::to_string(f[i]) + " N");
            }
        }
    }

    // 5. O6 温度限制
    if (state.hand.valid) {
        const auto& temp = state.hand.temperature;
        for (int i = 0; i < static_cast<int>(temp.size()); ++i) {
            if (temp[i] > config_.max_hand_temperature_c) {
                return Error::make(ErrorCategory::Safety, DeviceType::DexterousHand,
                                   "SafetySupervisor", 6,
                                   "O6 温度超限: 关节 " + std::to_string(i) + " = " +
                                       std::to_string(temp[i]) + " ℃");
            }
        }
    }

    return Error::ok();
}

void SafetySupervisor::request_stop(StopLevel level) {
    stop_level_ = level;
    if (level == StopLevel::EmergencyStop) {
        real_motion_enabled_ = false;  // 急停后必须人工复位
    }
}

StopLevel SafetySupervisor::current_stop_level() const {
    return stop_level_;
}

}  // namespace robotics::domain
