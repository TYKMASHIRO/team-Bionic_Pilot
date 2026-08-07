#pragma once

#include <mutex>
#include <string>
#include <unordered_map>

#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::domain {

/**
 * @brief 轨迹仓库实现（一期：内存存储 + 可选磁盘 CSV 导出）。
 */
class TrajectoryRepository : public ITrajectoryRepository {
public:
    Result load(const std::string& trajectory_id, Trajectory& out) override;
    Result save(const Trajectory& trajectory, std::string& out_id) override;
    std::vector<TrajectoryMeta> list() const override;

    /// 设置数据目录（保存到磁盘时使用）
    void set_data_dir(const std::string& dir);

private:
    mutable std::mutex mutex_;
    std::unordered_map<std::string, Trajectory> store_;
    std::string data_dir_;
};

}  // namespace robotics::domain
