#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/IDexterousHand.hpp"
#include "robotics/interfaces/IRobotArm.hpp"
#include "robotics/interfaces/ISafetySupervisor.hpp"
#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::domain {

/// 复现选项。
struct ReplayOptions {
    /// 时间轴缩放倍率：t_offset_ns / speed。1=原速，2=两倍速，0.5=半速。
    double speed = 1.0;
    bool dry_run = false;          ///< 只校验不运动
    /// 执行总时长上限（0=不设上限）
    std::chrono::milliseconds timeout{0};
    /// arm move_joint 速度比（0-1）
    double arm_speed_ratio = 0.2;
    /// 起点偏差容忍（rad，arm 关节最大偏差）
    double start_deviation_tol_rad = 0.1;
    /// 偏差超阈值时先 movej 到首点回起点；false 则直接拒绝
    bool align_start = true;
};

/// 复现结果报告。
struct ReplayReport {
    bool success = false;
    Error error;
    std::string failed_stage;

    std::size_t points_played = 0;
    std::size_t total_points = 0;
    std::chrono::milliseconds duration_ms{0};

    bool cancelled = false;
    bool timed_out = false;
    bool dry_run = false;
    bool safety_stopped = false;

    double start_deviation_rad = 0.0;  ///< 播放时实际起点偏差（arm）
    std::string final_summary;         ///< 设备最终状态摘要

    std::string trajectory_version;
    std::string config_version;
    std::string calibration_version;

    std::string to_string() const;
};

/**
 * @brief 轨迹复现执行器（阶段5）。
 *
 * 按统一时间轴逐点下发：
 *   arm  -> IRobotArm::move_joint(block=false)
 *   hand -> IDexterousHand::set_joint_positions
 * 每点推进到 t0 + t_offset_ns / speed；下发耗时超间隔时不补眠（跟随落后）。
 *
 * 前置：轨迹校验 + 设备在线 + 安全评估（真实运动许可）；起点偏差超阈值时
 * 先 movej 首点回起点（align_start）。
 *
 * 支持：Dry-run（不运动）、取消（cancel 回调每点前检查）、超时。
 */
class TrajectoryReplayer {
public:
    TrajectoryReplayer(std::shared_ptr<IRobotArm> arm,
                       std::shared_ptr<IDexterousHand> hand,
                       std::shared_ptr<ISafetySupervisor> safety,
                       std::shared_ptr<IClock> clock);

    /// 复现。cancel 为空或返回 false 表示未取消；非空时每点前检查。
    ReplayReport replay(const Trajectory& traj, const ReplayOptions& options,
                        const std::function<bool()>* cancel = nullptr) const;

private:
    std::shared_ptr<IRobotArm> arm_;
    std::shared_ptr<IDexterousHand> hand_;
    std::shared_ptr<ISafetySupervisor> safety_;
    std::shared_ptr<IClock> clock_;
};

}  // namespace robotics::domain
