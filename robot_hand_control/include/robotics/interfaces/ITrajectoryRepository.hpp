#pragma once

#include <string>
#include <vector>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/states/CombinedRobotState.hpp"

namespace robotics::domain {

/// 轨迹元数据
struct TrajectoryMeta {
    std::string trajectory_id;
    std::string name;
    std::string source_recording;  ///< 来源录制 session
    std::string calibration_id;
    std::string config_hash;
    std::string version;
    std::size_t sample_count = 0;
    double duration_s = 0.0;
    Timestamp created;
};

/// 轨迹点（统一时间轴：机械臂 + 灵巧手）
struct TrajectoryPoint {
    Timestamp timestamp;
    std::int64_t t_offset_ns = 0;  ///< 相对起点偏移（单调时间轴）
    RobotArmState arm;
    DexterousHandState hand;
    std::string event;  ///< 事件标记（可选）
};

/// 一条完整轨迹
struct Trajectory {
    TrajectoryMeta meta;
    std::vector<TrajectoryPoint> points;
};

/**
 * @brief 轨迹仓库：加载/保存/校验。
 */
class ITrajectoryRepository {
public:
    virtual ~ITrajectoryRepository() = default;

    /// 按 id 加载轨迹
    virtual Result load(const std::string& trajectory_id, Trajectory& out) = 0;

    /// 保存轨迹，返回生成的 id
    virtual Result save(const Trajectory& trajectory, std::string& out_id) = 0;

    /// 列出全部轨迹元数据
    virtual std::vector<TrajectoryMeta> list() const = 0;
};

}  // namespace robotics::domain
