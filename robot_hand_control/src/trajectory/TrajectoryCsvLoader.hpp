#pragma once

#include <string>

#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::domain {

/**
 * @brief 轨迹 CSV 读取器（阶段5）。
 *
 * 从录制目录读回轨迹：
 *   states.csv    解析为 TrajectoryPoint（复用 CsvStateColumns 110 列契约）
 *   events.csv    按 t_steady_ns 关联到最近的点（point.event）
 *   metadata.txt  校验 format_version（存在时）
 *
 * 加载后的点以相对首行偏移 t_offset_ns 构成统一时间轴（单调）。
 */
class TrajectoryCsvLoader {
public:
    /// 加载录制目录。recording_dir 为含 states.csv 的目录（如 data/recordings/<session>）。
    /// 失败返回带 Validation/Configuration 分类的 Result。
    static Result load_recording(const std::string& recording_dir,
                                 Trajectory& out);
};

}  // namespace robotics::domain
