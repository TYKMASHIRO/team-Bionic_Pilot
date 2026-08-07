#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>

#include "robotics/interfaces/IRobotArm.hpp"

// 厂商类型全部经 PIMPL 隔离：rm_* 只出现在 src/ 内，本头文件不暴露。
namespace robotics::realman {

/**
 * @brief RM75-6F 适配器。
 *
 * 封装睿尔曼 API2：
 * - 句柄生命周期（RAII）
 * - RM 错误码转换
 * - 7 关节数组转换（°→rad）
 * - 位姿/单位转换（m + rad）
 * - 拖动示教
 * - 六维力读取
 * - 实时状态读取
 * - 低速运动
 *
 * 真实运动需由上层 SafetySupervisor 显式授权（--enable-motion）。
 */
class RealManAdapter : public domain::IRobotArm {
public:
    RealManAdapter(std::string ip, int port, int thread_mode = 2);
    ~RealManAdapter() override;

    RealManAdapter(const RealManAdapter&) = delete;
    RealManAdapter& operator=(const RealManAdapter&) = delete;

    // ---- 连接管理 ----
    domain::Result connect() override;
    domain::Result disconnect() override;
    bool is_connected() const override;

    // ---- 使能 ----
    domain::Result enable() override;
    domain::Result disable() override;

    // ---- 状态 ----
    domain::RobotArmState get_state() const override;

    // ---- 运动（低速，须安全授权）----
    domain::Result move_joint(const domain::ArmJointVector& joint,
                              double speed_ratio, bool block) override;
    domain::Result move_pose(const domain::Pose& pose, double speed_ratio,
                             bool block) override;
    domain::Result move_linear(const domain::Pose& pose, double speed_ratio,
                               bool block) override;

    // ---- 停止 ----
    domain::Result stop() override;
    domain::Result emergency_stop() override;

    // ---- 拖动示教（★ 任务一核心）----
    domain::Result start_drag_teach(bool record_trajectory) override;
    domain::Result stop_drag_teach() override;
    domain::Result trajectory_to_origin(bool block) override;
    domain::Result replay_trajectory(bool block) override;
    domain::Result pause_trajectory() override;
    domain::Result continue_trajectory() override;
    domain::Result stop_trajectory() override;

    // ---- 坐标系 ----
    domain::Result set_tool_frame(const std::string& name,
                                  const domain::Pose& pose,
                                  double payload_kg) override;
    domain::Result set_work_frame(const std::string& name,
                                  const domain::Pose& pose) override;

    // ---- 力/诊断 ----
    domain::ForceTorque get_force_torque() const override;
    domain::Result clear_error() override;
    domain::DeviceHealth health_check() override;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;

    domain::Result require_connected() const;
};

}  // namespace robotics::realman
