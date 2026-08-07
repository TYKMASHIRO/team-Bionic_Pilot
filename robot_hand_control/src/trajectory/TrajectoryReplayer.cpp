#include "robotics/trajectory/TrajectoryReplayer.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <thread>

#include "robotics/trajectory/TrajectoryValidator.hpp"

namespace robotics::domain {

namespace {

std::string summary_of(const IRobotArm& arm, const IDexterousHand& hand) {
    const auto as = arm.get_state();
    const auto hs = hand.get_state();
    std::ostringstream ss;
    ss << "arm[" << (as.valid ? "ok" : "n/a");
    if (as.valid) {
        ss << " j=(" << as.joint_state.position[0];
        for (std::size_t i = 1; i < kArmDof; ++i) ss << "," << as.joint_state.position[i];
        ss << ")";
    }
    ss << "] hand[";
    if (hs.valid) {
        ss << hs.position[0];
        for (std::size_t i = 1; i < kHandDof; ++i) ss << "," << hs.position[i];
    } else {
        ss << "n/a";
    }
    ss << "]";
    return ss.str();
}

/// arm 当前关节与轨迹首点 arm 关节的最大绝对偏差（rad）。
double start_deviation(const IRobotArm& arm, const Trajectory& traj) {
    if (traj.points.empty()) return 0.0;
    const auto& first = traj.points.front();
    if (!first.arm.valid) return 0.0;
    const auto cur = arm.get_state();
    if (!cur.valid) return 0.0;
    double max_dev = 0.0;
    for (std::size_t i = 0; i < kArmDof; ++i) {
        max_dev = std::max(max_dev,
                           std::abs(cur.joint_state.position[i] -
                                    first.arm.joint_state.position[i]));
    }
    return max_dev;
}

}  // namespace

TrajectoryReplayer::TrajectoryReplayer(std::shared_ptr<IRobotArm> arm,
                                       std::shared_ptr<IDexterousHand> hand,
                                       std::shared_ptr<ISafetySupervisor> safety,
                                       std::shared_ptr<IClock> clock)
    : arm_(std::move(arm)),
      hand_(std::move(hand)),
      safety_(std::move(safety)),
      clock_(std::move(clock)) {}

std::string ReplayReport::to_string() const {
    std::ostringstream ss;
    ss << (success ? "成功" : "失败")
       << " stage=" << (failed_stage.empty() ? "-" : failed_stage)
       << " points=" << points_played << "/" << total_points
       << " duration=" << duration_ms.count() << "ms"
       << " cancelled=" << (cancelled ? "是" : "否")
       << " timed_out=" << (timed_out ? "是" : "否")
       << " dry_run=" << (dry_run ? "是" : "否");
    if (start_deviation_rad > 0.0) {
        ss << " start_dev=" << start_deviation_rad << "rad";
    }
    if (!error.is_ok()) ss << " err=" << error.to_string();
    if (!final_summary.empty()) ss << " final=" << final_summary;
    return ss.str();
}

ReplayReport TrajectoryReplayer::replay(const Trajectory& traj,
                                        const ReplayOptions& options,
                                        const std::function<bool()>* cancel) const {
    ReplayReport rep;
    rep.dry_run = options.dry_run;
    rep.total_points = traj.points.size();
    rep.trajectory_version = traj.meta.version;
    rep.calibration_version = traj.meta.calibration_id;

    const auto started = std::chrono::steady_clock::now();
    auto done = [&]() {
        rep.duration_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - started);
    };

    // 1. 轨迹校验（不运动）
    TrajectoryValidationReport vr;
    TrajectoryValidator::validate(traj, vr);
    if (!vr.valid) {
        rep.failed_stage = "validate";
        rep.error = Error::make(ErrorCategory::Validation, DeviceType::Combined,
                                "TrajectoryReplayer", 1, vr.to_string());
        done();
        return rep;
    }
    rep.points_played = 0;

    // 2. 设备在线
    if (!arm_ || !arm_->is_connected()) {
        rep.failed_stage = "devices_online";
        rep.error = Error::make(ErrorCategory::NotConnected, DeviceType::RobotArm,
                                "TrajectoryReplayer", 2, "机械臂未连接");
        done();
        return rep;
    }
    if (!hand_ || !hand_->is_connected()) {
        rep.failed_stage = "devices_online";
        rep.error = Error::make(ErrorCategory::NotConnected,
                                DeviceType::DexterousHand, "TrajectoryReplayer",
                                3, "灵巧手未连接");
        done();
        return rep;
    }

    // 3. 起点偏差（dry-run 也计算并报告）
    rep.start_deviation_rad = start_deviation(*arm_, traj);

    if (options.dry_run) {
        rep.failed_stage = "dry_run";
        rep.success = true;
        rep.final_summary = summary_of(*arm_, *hand_);
        done();
        return rep;
    }

    // 4. 安全评估（真实运动许可 + 状态新鲜）
    if (safety_) {
        CombinedRobotState combined;
        combined.arm = arm_->get_state();
        combined.hand = hand_->get_state();
        const Error se = safety_->evaluate(combined);
        if (!se.is_ok()) {
            rep.failed_stage = "safety";
            rep.safety_stopped = true;
            rep.error = se;
            done();
            return rep;
        }
    }

    // 5. 起点对齐：偏差超阈值先 movej 到首点
    if (rep.start_deviation_rad > options.start_deviation_tol_rad) {
        if (!options.align_start) {
            rep.failed_stage = "start_alignment";
            rep.error = Error::make(ErrorCategory::Safety, DeviceType::RobotArm,
                                    "TrajectoryReplayer", 4,
                                    "起点偏差超阈值且 align_start=false: " +
                                        std::to_string(rep.start_deviation_rad));
            done();
            return rep;
        }
        const auto& first = traj.points.front();
        if (first.arm.valid) {
            Result r = arm_->move_joint(first.arm.joint_state.position,
                                        options.arm_speed_ratio, /*block=*/true);
            if (!r.success) {
                rep.failed_stage = "start_alignment";
                rep.error = r.error;
                done();
                return rep;
            }
        }
    }

    // 6. 播放
    const double speed =
        options.speed > 0.0 ? options.speed : 1.0;
    const auto t0 = std::chrono::steady_clock::now();
    const auto deadline =
        options.timeout.count() > 0
            ? t0 + options.timeout
            : std::chrono::steady_clock::time_point::max();

    std::size_t i = 0;
    for (; i < traj.points.size(); ++i) {
        if (cancel && (*cancel)()) {
            rep.cancelled = true;
            break;
        }
        if (std::chrono::steady_clock::now() > deadline) {
            rep.timed_out = true;
            break;
        }

        const auto& p = traj.points[i];
        const bool is_last_point = (i + 1 == traj.points.size());

        if (p.arm.valid) {
            // 末点阻塞到位；其余非阻塞沿统一时间轴推进
            const bool block = is_last_point;
            Result r = arm_->move_joint(p.arm.joint_state.position,
                                        options.arm_speed_ratio, block);
            if (!r.success) {
                rep.failed_stage = "play_arm";
                rep.error = r.error;
                done();
                return rep;
            }
        }
        if (p.hand.valid) {
            Result r = hand_->set_joint_positions(p.hand.position);
            if (!r.success) {
                rep.failed_stage = "play_hand";
                rep.error = r.error;
                done();
                return rep;
            }
        }
        ++rep.points_played;

        // 推进到目标时刻（下发耗时超间隔则直接进入下一步，不补眠）
        const std::int64_t target_ns =
            static_cast<std::int64_t>(static_cast<double>(p.t_offset_ns) / speed);
        const auto target =
            t0 + std::chrono::nanoseconds(target_ns);
        const auto now = std::chrono::steady_clock::now();
        if (target > now) {
            std::this_thread::sleep_until(target);
        }
    }

    rep.failed_stage = rep.cancelled || rep.timed_out ? "stopped" : "replay";
    rep.success = !rep.cancelled && !rep.timed_out;
    rep.final_summary = summary_of(*arm_, *hand_);
    done();
    return rep;
}

}  // namespace robotics::domain
