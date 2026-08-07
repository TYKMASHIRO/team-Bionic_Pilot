#pragma once

#include <atomic>
#include <mutex>
#include <thread>

#include "robotics/interfaces/IRobotArm.hpp"

namespace robotics::mock {

/**
 * @brief Mock 机械臂。
 * 模拟连接/断开/运动/到位/拖动示教/轨迹复现/故障注入。
 * 无真实硬件时作为测试与仿真基础。
 */
class MockRobotArm : public domain::IRobotArm {
public:
    MockRobotArm() = default;
    ~MockRobotArm() override;

    // ---- 故障注入（测试用）----
    void inject_fault(bool enable) { fault_ = enable; }
    void set_starting_pose(const domain::Pose& pose) { start_pose_ = pose; }

    // IRobotArm
    domain::Result connect() override;
    domain::Result disconnect() override;
    bool is_connected() const override;

    domain::Result enable() override;
    domain::Result disable() override;

    domain::RobotArmState get_state() const override;

    domain::Result move_joint(const domain::ArmJointVector& joint,
                              double speed_ratio, bool block) override;
    domain::Result move_pose(const domain::Pose& pose, double speed_ratio,
                             bool block) override;
    domain::Result move_linear(const domain::Pose& pose, double speed_ratio,
                               bool block) override;

    domain::Result stop() override;
    domain::Result emergency_stop() override;

    domain::Result start_drag_teach(bool record_trajectory) override;
    domain::Result stop_drag_teach() override;

    domain::Result trajectory_to_origin(bool block) override;
    domain::Result replay_trajectory(bool block) override;
    domain::Result pause_trajectory() override;
    domain::Result continue_trajectory() override;
    domain::Result stop_trajectory() override;

    domain::Result set_tool_frame(const std::string& name,
                                  const domain::Pose& pose,
                                  double payload_kg) override;
    domain::Result set_work_frame(const std::string& name,
                                  const domain::Pose& pose) override;

    domain::ForceTorque get_force_torque() const override;
    domain::Result clear_error() override;
    domain::DeviceHealth health_check() override;

private:
    void simulate_motion(const domain::ArmJointVector& target);

    mutable std::mutex mutex_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> enabled_{false};
    std::atomic<bool> fault_{false};
    std::atomic<bool> drag_teaching_{false};
    std::atomic<bool> replaying_{false};

    domain::ArmJointVector current_joint_{};
    domain::Pose current_pose_;
    domain::Pose start_pose_;
    domain::ArmMotionState motion_state_{domain::ArmMotionState::Idle};
    domain::ForceTorque force_;
    bool recorded_ = false;
};

}  // namespace robotics::mock
