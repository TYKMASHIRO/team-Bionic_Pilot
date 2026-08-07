#pragma once

#include <string>
#include <vector>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::domain {

/// 校验结果：valid=false 时 errors 含首个失败原因（可多错误）。
struct TrajectoryValidationReport {
    bool valid = false;
    std::vector<std::string> errors;
    std::vector<std::string> warnings;

    std::size_t sample_count = 0;
    double duration_s = 0.0;
    bool monotonic = false;  ///< t_offset_ns 单调非递减
    bool finite = false;     ///< 数值均有限（无 NaN/Inf）
    bool in_range = false;   ///< 关节位置在软限位内
    bool dof_ok = false;     ///< 关节数匹配（arm 7 / hand 6）
    std::string to_string() const;
};

/// 范围软限位（默认宽松；测试可注入更紧限位）。
struct TrajectoryRangeLimits {
    double arm_joint_min_rad = -3.5;
    double arm_joint_max_rad = 3.5;
    double hand_min_raw = 0.0;
    double hand_max_raw = 255.0;
};

/**
 * @brief 轨迹校验器：格式/版本/关节数/单调/NaN/范围。
 * 校验不运动、不读设备，可安全用于 Dry-run。
 */
class TrajectoryValidator {
public:
    /// 校验轨迹并填充报告。不抛异常。
    static void validate(const Trajectory& traj, TrajectoryValidationReport& out,
                         const TrajectoryRangeLimits& limits = {});
};

}  // namespace robotics::domain
