// RealManAdapter —— 阶段2 完整实现。
// 当前为骨架：所有接口返回 Unsupported，保证工程可编译。
// 本文件避免编译时链接 RM 库，具体实现见阶段2。
#include "drivers/realman/include/RealManAdapter.hpp"

namespace robotics::realman {

using namespace robotics::domain;

RealManAdapter::RealManAdapter(std::string ip, int port)
    : ip_(std::move(ip)), port_(port) {}

RealManAdapter::~RealManAdapter() {
    disconnect();
}

Result RealManAdapter::connect() {
    return Result::fail(Error::make(ErrorCategory::Unsupported, DeviceType::RobotArm,
        "RealManAdapter", 1, "RM75 驱动阶段2实现"));
}
Result RealManAdapter::disconnect() { return Result::ok(); }
bool RealManAdapter::is_connected() const { return false; }
Result RealManAdapter::enable() {
    return Result::fail(Error::make(ErrorCategory::Unsupported, DeviceType::RobotArm,
        "RealManAdapter", 2, "阶段2实现"));
}
Result RealManAdapter::disable() { return Result::ok(); }
RobotArmState RealManAdapter::get_state() const { return RobotArmState{}; }

Result RealManAdapter::move_joint(const ArmJointVector&, double, bool) {
    return unsupported();
}
Result RealManAdapter::move_pose(const Pose&, double, bool) { return unsupported(); }
Result RealManAdapter::move_linear(const Pose&, double, bool) { return unsupported(); }
Result RealManAdapter::stop() { return unsupported(); }
Result RealManAdapter::emergency_stop() { return unsupported(); }
Result RealManAdapter::start_drag_teach(bool) { return unsupported(); }
Result RealManAdapter::stop_drag_teach() { return unsupported(); }
Result RealManAdapter::trajectory_to_origin(bool) { return unsupported(); }
Result RealManAdapter::replay_trajectory(bool) { return unsupported(); }
Result RealManAdapter::pause_trajectory() { return unsupported(); }
Result RealManAdapter::continue_trajectory() { return unsupported(); }
Result RealManAdapter::stop_trajectory() { return unsupported(); }
Result RealManAdapter::set_tool_frame(const std::string&, const Pose&, double) {
    return unsupported();
}
Result RealManAdapter::set_work_frame(const std::string&, const Pose&) {
    return unsupported();
}
ForceTorque RealManAdapter::get_force_torque() const { return ForceTorque{}; }
Result RealManAdapter::clear_error() { return unsupported(); }
DeviceHealth RealManAdapter::health_check() {
    DeviceHealth h;
    h.device = DeviceType::RobotArm;
    h.driver_loaded = true;
    h.sdk_version = "1.1.6";
    h.level = HealthLevel::Fault;
    h.summary = "RealMan driver not implemented (stage 2)";
    return h;
}

Result RealManAdapter::unsupported() const {
    return Result::fail(Error::make(ErrorCategory::Unsupported, DeviceType::RobotArm,
        "RealManAdapter", 99, "该能力阶段2实现"));
}

}  // namespace robotics::realman
