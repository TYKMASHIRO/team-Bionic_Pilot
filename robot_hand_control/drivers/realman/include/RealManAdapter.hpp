#pragma once

#include <string>

#include "robotics/interfaces/IRobotArm.hpp"

namespace robotics::realman {

/**
 * @brief RM75-6F 适配器（阶段2完整实现）。
 * 当前为骨架：返回 Unsupported，保证工程可编译。
 */
class RealManAdapter : public domain::IRobotArm {
public:
    RealManAdapter(std::string ip, int port);
    ~RealManAdapter() override;

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
    domain::Result unsupported() const;

    std::string ip_;
    int port_ = 8080;
    // 阶段2: rm_robot_handle* handle_ = nullptr;（仅存在于 .cpp）
};

}  // namespace robotics::realman
