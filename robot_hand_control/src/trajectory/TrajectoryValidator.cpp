#include "robotics/trajectory/TrajectoryValidator.hpp"

#include <cmath>
#include <sstream>

#include "robotics/services/RecordingMetadata.hpp"

namespace robotics::domain {

namespace {
bool is_finite(double v) { return std::isfinite(v); }
}  // namespace

std::string TrajectoryValidationReport::to_string() const {
    std::ostringstream ss;
    ss << (valid ? "通过" : "失败") << " | 点数=" << sample_count
       << " 时长=" << duration_s << "s"
       << " 单调=" << (monotonic ? "是" : "否")
       << " 有限=" << (finite ? "是" : "否")
       << " 范围=" << (in_range ? "是" : "否")
       << " 关节数=" << (dof_ok ? "匹配" : "不匹配");
    if (!errors.empty()) {
        ss << " | 错误:";
        for (const auto& e : errors) ss << " [" << e << "]";
    }
    if (!warnings.empty()) {
        ss << " | 警告:";
        for (const auto& w : warnings) ss << " [" << w << "]";
    }
    return ss.str();
}

void TrajectoryValidator::validate(const Trajectory& traj,
                                   TrajectoryValidationReport& out,
                                   const TrajectoryRangeLimits& limits) {
    out = TrajectoryValidationReport{};
    out.sample_count = traj.points.size();
    out.duration_s = traj.meta.duration_s;

    // 1. 非空
    if (traj.points.empty()) {
        out.errors.emplace_back("轨迹为空（无状态点）");
        return;
    }

    // 2. 版本
    if (!traj.meta.version.empty()) {
        const int v = std::atoi(traj.meta.version.c_str());
        if (v != kRecordingFormatVersion) {
            out.errors.emplace_back("版本不匹配: " + traj.meta.version);
            return;
        }
    }

    // 3. 关节数（强类型下恒匹配，但保留检查以防数据结构退化）
    bool dof_ok = true;
    for (std::size_t i = 0; i < traj.points.size(); ++i) {
        const auto& p = traj.points[i];
        if (p.arm.valid &&
            p.arm.joint_state.position.size() != kArmDof) {
            dof_ok = false;
            break;
        }
        if (p.hand.valid && p.hand.position.size() != kHandDof) {
            dof_ok = false;
            break;
        }
    }
    out.dof_ok = dof_ok;
    if (!dof_ok) {
        out.errors.emplace_back("关节数不匹配（arm 应为 7 / hand 应为 6）");
        return;
    }

    // 4. 时间戳单调（t_offset_ns 非递减）
    bool monotonic = true;
    std::int64_t prev = traj.points.front().t_offset_ns;
    for (std::size_t i = 1; i < traj.points.size(); ++i) {
        const std::int64_t cur = traj.points[i].t_offset_ns;
        if (cur < prev) {
            monotonic = false;
            out.errors.emplace_back("时间戳非单调: 点 " + std::to_string(i - 1) +
                                    " (" + std::to_string(prev) + ") -> 点 " +
                                    std::to_string(i) + " (" + std::to_string(cur) +
                                    ") ns");
            break;
        }
        prev = cur;
    }
    out.monotonic = monotonic;
    if (!monotonic) return;

    // 5. NaN/Inf + 范围
    bool finite = true;
    bool in_range = true;
    std::size_t first_nonfinite = 0, first_outrange = 0;
    auto check_finite = [&](double v, std::size_t pi, const char* what) {
        if (!is_finite(v)) {
            finite = false;
            if (first_nonfinite == 0) first_nonfinite = pi + 1;
        }
    };
    auto check_range = [&](double v, double lo, double hi, std::size_t pi,
                           const char* what) {
        if (!(v >= lo && v <= hi)) {
            in_range = false;
            if (first_outrange == 0) first_outrange = pi + 1;
        }
    };

    for (std::size_t i = 0; i < traj.points.size(); ++i) {
        const auto& p = traj.points[i];
        if (p.arm.valid) {
            const auto& js = p.arm.joint_state;
            for (std::size_t j = 0; j < kArmDof; ++j) {
                check_finite(js.position[j], i, "arm_j_pos");
                check_finite(js.velocity[j], i, "arm_j_vel");
                check_finite(js.current[j], i, "arm_j_cur");
                check_finite(js.temperature[j], i, "arm_j_temp");
                check_range(js.position[j], limits.arm_joint_min_rad,
                            limits.arm_joint_max_rad, i, "arm_j_pos");
            }
            check_finite(p.arm.tcp_pose.position.x, i, "arm_tcp_x");
            check_finite(p.arm.tcp_pose.position.y, i, "arm_tcp_y");
            check_finite(p.arm.tcp_pose.position.z, i, "arm_tcp_z");
            for (std::size_t j = 0; j < 6; ++j) {
                check_finite(p.arm.force_torque.to_vector()[j], i, "arm_ft");
            }
        }
        if (p.hand.valid) {
            for (std::size_t j = 0; j < kHandDof; ++j) {
                check_finite(p.hand.position[j], i, "hand_pos");
                check_finite(p.hand.temperature[j], i, "hand_temp");
                check_range(p.hand.position[j], limits.hand_min_raw,
                            limits.hand_max_raw, i, "hand_pos");
            }
        }
    }
    out.finite = finite;
    out.in_range = in_range;

    if (!finite) {
        out.errors.emplace_back("存在非有限数值（NaN/Inf），首现于点 " +
                                std::to_string(first_nonfinite));
    }
    if (!in_range) {
        out.errors.emplace_back("关节位置超出软限位，首现于点 " +
                                std::to_string(first_outrange));
    }

    // 双设备在场比（信息性）
    if (!traj.points.empty()) {
        std::size_t arm_ok = 0, hand_ok = 0;
        for (const auto& p : traj.points) {
            if (p.arm.valid) ++arm_ok;
            if (p.hand.valid) ++hand_ok;
        }
        if (hand_ok == 0) {
            out.warnings.emplace_back("轨迹不含有效灵巧手状态");
        }
        if (arm_ok == 0) {
            out.warnings.emplace_back("轨迹不含有效机械臂状态");
        }
    }

    out.valid = out.errors.empty();
}

}  // namespace robotics::domain
