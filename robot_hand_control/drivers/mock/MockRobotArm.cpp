#include "drivers/mock/MockRobotArm.hpp"

#include <chrono>
#include <cmath>
#include <thread>

namespace robotics::mock {

using namespace robotics::domain;

MockRobotArm::~MockRobotArm() {
    disconnect();
}

Result MockRobotArm::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_) {
        return Result::fail(Error::make(ErrorCategory::Communication,
                                        DeviceType::RobotArm, "MockRobotArm", 1,
                                        "注入通信故障"));
    }
    connected_ = true;
    enabled_ = true;
    motion_state_ = ArmMotionState::Idle;
    current_pose_ = start_pose_;
    return Result::ok();
}

Result MockRobotArm::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    enabled_ = false;
    return Result::ok();
}

bool MockRobotArm::is_connected() const {
    return connected_;
}

Result MockRobotArm::enable() {
    enabled_ = true;
    return Result::ok();
}

Result MockRobotArm::disable() {
    enabled_ = false;
    return Result::ok();
}

RobotArmState MockRobotArm::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    RobotArmState st;
    st.joint_state.position = current_joint_;
    st.joint_state.valid = connected_ && enabled_;
    st.tcp_pose = current_pose_;
    st.force_torque = force_;
    st.motion_state = motion_state_;
    st.reached_target = (motion_state_ == ArmMotionState::Idle);
    st.valid = connected_ && enabled_;
    st.fresh = true;
    st.timestamp = make_timestamp();
    st.sequence = 0;  // 阶段4 由 StateStore 统一打序号
    return st;
}

void MockRobotArm::simulate_motion(const ArmJointVector& target) {
    if (!connected_) return;
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Moving;
    // 简单模拟：立即到位（真实模拟可加插值）
    current_joint_ = target;
    current_pose_.position.x += 0.01;
    motion_state_ = ArmMotionState::Idle;
}

Result MockRobotArm::move_joint(const ArmJointVector& joint, double speed_ratio,
                                bool block) {
    (void)speed_ratio;
    (void)block;
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::RobotArm, "MockRobotArm", 2, "未连接"));
    if (fault_) return Result::fail(Error::make(
        ErrorCategory::Device, DeviceType::RobotArm, "MockRobotArm", 3, "注入设备故障"));
    simulate_motion(joint);
    return Result::ok();
}

Result MockRobotArm::move_pose(const Pose& pose, double speed_ratio, bool block) {
    (void)speed_ratio;
    (void)block;
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::RobotArm, "MockRobotArm", 2, "未连接"));
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = pose;
    motion_state_ = ArmMotionState::Idle;
    return Result::ok();
}

Result MockRobotArm::move_linear(const Pose& pose, double speed_ratio, bool block) {
    return move_pose(pose, speed_ratio, block);
}


Result MockRobotArm::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Stopped;
    return Result::ok();
}

Result MockRobotArm::emergency_stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::EmergencyStopped;
    return Result::ok();
}

Result MockRobotArm::start_drag_teach(bool record_trajectory) {
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::RobotArm, "MockRobotArm", 2, "未连接"));
    drag_teaching_ = true;
    recorded_ = record_trajectory;
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::DragTeaching;
    return Result::ok();
}

Result MockRobotArm::stop_drag_teach() {
    drag_teaching_ = false;
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Idle;
    return Result::ok();
}

Result MockRobotArm::trajectory_to_origin(bool block) {
    (void)block;
    if (!recorded_) return Result::fail(Error::make(
        ErrorCategory::Validation, DeviceType::RobotArm, "MockRobotArm", 4,
        "无记录轨迹"));
    std::lock_guard<std::mutex> lock(mutex_);
    current_pose_ = start_pose_;
    motion_state_ = ArmMotionState::Idle;
    return Result::ok();
}

Result MockRobotArm::replay_trajectory(bool block) {
    (void)block;
    if (!recorded_) return Result::fail(Error::make(
        ErrorCategory::Validation, DeviceType::RobotArm, "MockRobotArm", 4,
        "无记录轨迹"));
    replaying_ = true;
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Replaying;
    // Mock：模拟一小段复现
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    motion_state_ = ArmMotionState::Idle;
    replaying_ = false;
    return Result::ok();
}

Result MockRobotArm::pause_trajectory() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Paused;
    return Result::ok();
}

Result MockRobotArm::continue_trajectory() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Replaying;
    return Result::ok();
}

Result MockRobotArm::stop_trajectory() {
    std::lock_guard<std::mutex> lock(mutex_);
    motion_state_ = ArmMotionState::Stopped;
    replaying_ = false;
    return Result::ok();
}

Result MockRobotArm::set_tool_frame(const std::string& name, const Pose& pose,
                                    double payload_kg) {
    (void)name;
    (void)pose;
    (void)payload_kg;
    return Result::ok();
}

Result MockRobotArm::set_work_frame(const std::string& name, const Pose& pose) {
    (void)name;
    (void)pose;
    return Result::ok();
}

ForceTorque MockRobotArm::get_force_torque() const {
    return force_;
}

Result MockRobotArm::clear_error() {
    fault_ = false;
    return Result::ok();
}

DeviceHealth MockRobotArm::health_check() {
    DeviceHealth h;
    h.device = DeviceType::RobotArm;
    h.online = connected_;
    h.driver_loaded = true;
    h.sdk_version = "mock";
    h.firmware_version = "mock";
    h.level = (connected_ && !fault_) ? HealthLevel::Ok : HealthLevel::Fault;
    h.summary = connected_ ? (fault_ ? "mock arm (fault injected)"
                                     : "mock arm ok")
                           : "mock arm disconnected";
    return h;
}

}  // namespace robotics::mock
