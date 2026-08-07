#include "drivers/realman/include/RealManAdapter.hpp"

#include <algorithm>
#include <cmath>

#include <rm_define.h>
#include <rm_interface.h>

#include "drivers/realman/include/RmErrorMap.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"

namespace robotics::realman {

namespace {
constexpr double kRad2Deg = 180.0 / 3.14159265358979323846;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr int kRmV = 20;      // 低速默认速度（关节 0-100）
constexpr int kRmR = 0;       // 无过渡
constexpr int kTrajDisconnect = 0;
auto& log() { return robotics::infra::Logger::instance(); }

domain::ArmJointVector rm_joints_to_rad(const float* j) {
    domain::ArmJointVector out{};
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        out[i] = j[i] * kDeg2Rad;
    }
    return out;
}

float* rad_to_rm_joints(const domain::ArmJointVector& v, float out[domain::kArmDof]) {
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        out[i] = static_cast<float>(v[i] * kRad2Deg);
    }
    return out;
}

domain::Pose rm_pose_to_domain(const rm_pose_t& p) {
    domain::Pose out;
    out.position.x = p.position.x;
    out.position.y = p.position.y;
    out.position.z = p.position.z;
    // RM 同时提供四元数与欧拉角；优先用欧拉角（rad）
    out.orientation.rx = p.euler.rx;
    out.orientation.ry = p.euler.ry;
    out.orientation.rz = p.euler.rz;
    return out;
}

rm_pose_t domain_pose_to_rm(const domain::Pose& p) {
    rm_pose_t out{};
    out.position.x = static_cast<float>(p.position.x);
    out.position.y = static_cast<float>(p.position.y);
    out.position.z = static_cast<float>(p.position.z);
    // 欧拉角直接给（rad）；四元数置 0 由厂商忽略欧拉角模式
    out.euler.rx = static_cast<float>(p.orientation.rx);
    out.euler.ry = static_cast<float>(p.orientation.ry);
    out.euler.rz = static_cast<float>(p.orientation.rz);
    return out;
}
}  // namespace

// ---------------------------------------------------------------------------
// PIMPL 实现
// ---------------------------------------------------------------------------
class RealManAdapter::Impl {
public:
    Impl(std::string ip, int port, int thread_mode)
        : ip_(std::move(ip)), port_(port), thread_mode_(thread_mode) {}

    ~Impl() {
        if (handle_) {
            rm_delete_robot_arm(handle_);
            handle_ = nullptr;
        }
        if (sdk_initialized_) {
            rm_destroy();
            sdk_initialized_ = false;
        }
    }

    std::string ip_;
    int port_ = 8080;
    int thread_mode_ = 2;
    rm_robot_handle* handle_ = nullptr;
    bool sdk_initialized_ = false;
    std::atomic<bool> connected_{false};
    mutable std::mutex mutex_;
};

// ---------------------------------------------------------------------------
// 生命周期
// ---------------------------------------------------------------------------
RealManAdapter::RealManAdapter(std::string ip, int port, int thread_mode)
    : impl_(std::make_unique<Impl>(std::move(ip), port, thread_mode)) {}

RealManAdapter::~RealManAdapter() = default;

domain::Result RealManAdapter::connect() {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->connected_) return domain::Result::ok();

    // 1. 初始化 SDK（线程模式）
    if (!impl_->sdk_initialized_) {
        const int rc = rm_init(static_cast<rm_thread_mode_e>(impl_->thread_mode_));
        if (rc != 0) {
            return domain::Result::fail(rm_to_error(
                rc, domain::DeviceType::RobotArm, "RealManAdapter", "rm_init"));
        }
        impl_->sdk_initialized_ = true;
    }

    // 2. 创建机械臂句柄
    impl_->handle_ = rm_create_robot_arm(impl_->ip_.c_str(), impl_->port_);
    if (impl_->handle_ == nullptr) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Communication, domain::DeviceType::RobotArm,
            "RealManAdapter", 100, "连接失败: " + impl_->ip_ + ":" +
                                      std::to_string(impl_->port_),
            domain::Severity::Error, true));
    }

    impl_->connected_ = true;
    log().info("RealManAdapter", "connected to " + impl_->ip_ + ":" +
                                     std::to_string(impl_->port_));
    return domain::Result::ok();
}

void* RealManAdapter::native_handle() const {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    return impl_->handle_;
}

domain::Result RealManAdapter::disconnect() {
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (impl_->handle_) {
        rm_delete_robot_arm(impl_->handle_);
        impl_->handle_ = nullptr;
    }
    impl_->connected_ = false;
    return domain::Result::ok();
}

bool RealManAdapter::is_connected() const {
    return impl_->connected_;
}

domain::Result RealManAdapter::require_connected() const {
    if (!impl_->connected_) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::NotConnected, domain::DeviceType::RobotArm,
            "RealManAdapter", 101, "机械臂未连接"));
    }
    return domain::Result::ok();
}

domain::Result RealManAdapter::enable() {
    // RM API 通过连接即使能；此接口保留语义，返回 ok
    return require_connected();
}

domain::Result RealManAdapter::disable() {
    // RM 无独立 disable 语义；保留接口
    return domain::Result::ok();
}

// ---------------------------------------------------------------------------
// 状态读取
// ---------------------------------------------------------------------------
domain::RobotArmState RealManAdapter::get_state() const {
    domain::RobotArmState st;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->connected_ || !impl_->handle_) {
        st.valid = false;
        return st;
    }

    // 全量状态（电流/使能/温度/电压/错误码）
    rm_arm_all_state_t all{};
    int rc = rm_get_arm_all_state(impl_->handle_, &all);
    if (rc != 0) {
        st.valid = false;
        return st;
    }

    // 当前状态（位姿 + 关节角 + 系统错误）
    rm_current_arm_state_t cur{};
    rc = rm_get_current_arm_state(impl_->handle_, &cur);
    if (rc != 0) {
        st.valid = false;
        return st;
    }

    // 关节状态（角度→rad）
    const domain::ArmJointVector joints_rad = rm_joints_to_rad(cur.joint);
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        st.joint_state.position[i] = joints_rad[i];
        st.joint_state.current[i] = all.joint_current[i] / 1000.0;  // mA→A
        st.joint_state.temperature[i] = all.joint_temperature[i];
        st.joint_state.velocity[i] = 0.0;  // 拉模式无速度数组，留 0
        st.joint_state.enabled[i] = all.joint_en_flag[i] != 0;
        st.joint_state.error_code[i] =
            static_cast<std::uint16_t>(all.joint_err_code[i]);
    }
    st.joint_state.valid = true;

    // TCP 位姿
    st.tcp_pose = rm_pose_to_domain(cur.pose);

    // 系统错误码
    st.system_errors.assign(cur.err.err, cur.err.err + cur.err.err_len);

    // 六维力
    rm_force_data_t force{};
    if (rm_get_force_data(impl_->handle_, &force) == 0) {
        st.force_torque.force = {force.force_data[0], force.force_data[1],
                                 force.force_data[2]};
        st.force_torque.torque = {force.force_data[3], force.force_data[4],
                                  force.force_data[5]};
    }

    // 运动状态推断：有系统错误→Fault；否则 Idle（拉模式无法精确区分）
    st.motion_state = st.system_errors.empty() ? domain::ArmMotionState::Idle
                                               : domain::ArmMotionState::Stopped;
    st.reached_target = st.system_errors.empty();

    st.valid = true;
    st.fresh = true;
    st.timestamp = domain::make_timestamp();
    st.sequence = 0;  // 阶段4 由 StateStore 统一打序号
    return st;
}

// ---------------------------------------------------------------------------
// 运动（低速）
// ---------------------------------------------------------------------------
domain::Result RealManAdapter::move_joint(const domain::ArmJointVector& joint,
                                          double speed_ratio, bool block) {
    auto r = require_connected();
    if (!r.success) return r;

    float joints[domain::kArmDof];
    rad_to_rm_joints(joint, joints);
    const int speed = std::max(1, std::min(100, static_cast<int>(speed_ratio * 100)));
    const int rc = rm_movej(impl_->handle_, joints, speed, kRmR,
                            kTrajDisconnect, block ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_movej");
}

domain::Result RealManAdapter::move_pose(const domain::Pose& pose,
                                         double speed_ratio, bool block) {
    auto r = require_connected();
    if (!r.success) return r;

    const int speed = std::max(1, std::min(100, static_cast<int>(speed_ratio * 100)));
    const rm_pose_t rm_pose = domain_pose_to_rm(pose);
    const int rc = rm_movej_p(impl_->handle_, rm_pose, speed, kRmR,
                              kTrajDisconnect, block ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_movej_p");
}

domain::Result RealManAdapter::move_linear(const domain::Pose& pose,
                                           double speed_ratio, bool block) {
    auto r = require_connected();
    if (!r.success) return r;

    const int speed = std::max(1, std::min(100, static_cast<int>(speed_ratio * 100)));
    const rm_pose_t rm_pose = domain_pose_to_rm(pose);
    const int rc = rm_movel(impl_->handle_, rm_pose, speed, kRmR,
                            kTrajDisconnect, block ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_movel");
}

domain::Result RealManAdapter::stop() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_set_arm_slow_stop(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_set_arm_slow_stop");
}

domain::Result RealManAdapter::emergency_stop() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_set_arm_stop(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_set_arm_stop");
}

// ---------------------------------------------------------------------------
// 拖动示教（★ 任务一核心）
// ---------------------------------------------------------------------------
domain::Result RealManAdapter::start_drag_teach(bool record_trajectory) {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_start_drag_teach(impl_->handle_, record_trajectory ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_start_drag_teach");
}

domain::Result RealManAdapter::stop_drag_teach() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_stop_drag_teach(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_stop_drag_teach");
}

domain::Result RealManAdapter::trajectory_to_origin(bool block) {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_drag_trajectory_origin(impl_->handle_, block ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_drag_trajectory_origin");
}

domain::Result RealManAdapter::replay_trajectory(bool block) {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_run_drag_trajectory(impl_->handle_, block ? 1 : 0);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_run_drag_trajectory");
}

domain::Result RealManAdapter::pause_trajectory() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_pause_drag_trajectory(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_pause_drag_trajectory");
}

domain::Result RealManAdapter::continue_trajectory() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_continue_drag_trajectory(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_continue_drag_trajectory");
}

domain::Result RealManAdapter::stop_trajectory() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_stop_drag_trajectory(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_stop_drag_trajectory");
}

// ---------------------------------------------------------------------------
// 坐标系
// ---------------------------------------------------------------------------
domain::Result RealManAdapter::set_tool_frame(const std::string& name,
                                              const domain::Pose& pose,
                                              double payload_kg) {
    auto r = require_connected();
    if (!r.success) return r;

    rm_frame_t frame{};
    std::snprintf(frame.frame_name, sizeof(frame.frame_name), "%s", name.c_str());
    frame.pose = domain_pose_to_rm(pose);
    frame.payload = static_cast<float>(payload_kg);
    const int rc = rm_set_manual_tool_frame(impl_->handle_, frame);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_set_manual_tool_frame");
}

domain::Result RealManAdapter::set_work_frame(const std::string& name,
                                              const domain::Pose& pose) {
    auto r = require_connected();
    if (!r.success) return r;

    const rm_pose_t rm_pose = domain_pose_to_rm(pose);
    const int rc = rm_set_manual_work_frame(impl_->handle_, name.c_str(), rm_pose);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_set_manual_work_frame");
}

// ---------------------------------------------------------------------------
// 力/诊断
// ---------------------------------------------------------------------------
domain::ForceTorque RealManAdapter::get_force_torque() const {
    domain::ForceTorque ft;
    std::lock_guard<std::mutex> lock(impl_->mutex_);
    if (!impl_->connected_ || !impl_->handle_) return ft;

    rm_force_data_t force{};
    if (rm_get_force_data(impl_->handle_, &force) == 0) {
        ft.force = {force.force_data[0], force.force_data[1], force.force_data[2]};
        ft.torque = {force.force_data[3], force.force_data[4], force.force_data[5]};
    }
    return ft;
}

domain::Result RealManAdapter::clear_error() {
    auto r = require_connected();
    if (!r.success) return r;
    const int rc = rm_clear_system_err(impl_->handle_);
    return rm_to_result(rc, domain::DeviceType::RobotArm, "RealManAdapter",
                       "rm_clear_system_err");
}

domain::DeviceHealth RealManAdapter::health_check() {
    domain::DeviceHealth h;
    h.device = domain::DeviceType::RobotArm;
    h.online = impl_->connected_;
    h.driver_loaded = true;

    if (!impl_->connected_ || !impl_->handle_) {
        h.level = domain::HealthLevel::Fault;
        h.summary = "RM75 not connected";
        return h;
    }

    h.sdk_version = "1.1.6";

    // 读取软件信息
    rm_arm_software_version_t sw{};
    if (rm_get_arm_software_info(impl_->handle_, &sw) == 0) {
        h.firmware_version = sw.product_version;
        h.level = domain::HealthLevel::Ok;
        h.summary = std::string("RM75 ") + sw.product_version;
    } else {
        h.level = domain::HealthLevel::Degraded;
        h.summary = "RM75 connected, firmware read failed";
    }
    return h;
}

}  // namespace robotics::realman
