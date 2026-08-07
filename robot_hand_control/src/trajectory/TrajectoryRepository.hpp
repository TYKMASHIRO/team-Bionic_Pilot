#pragma once

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::domain {

/**
 * @brief 轨迹仓库实现（内存缓存 + 磁盘持久化）。
 *
 * 磁盘布局：<data_dir>/<trajectory_id>/
 *   states.csv   110 列（CsvStateColumns 序列化，与录制/加载共用契约）
 *   events.csv   事件行（可选，复用录制事件格式）
 *   metadata.txt 录制格式版本 / config_hash / calibration_ref
 *   manifest.txt 轨迹专属字段（trajectory_id / name / source_recording / created）
 *
 * 加载优先内存缓存，未命中时回退磁盘（TrajectoryCsvLoader 复用解析逻辑）。
 * CLI 进程隔离：不同进程通过磁盘资产共享轨迹。
 */
class TrajectoryRepository : public ITrajectoryRepository {
public:
    Result load(const std::string& trajectory_id, Trajectory& out) override;
    Result save(const Trajectory& trajectory, std::string& out_id) override;
    std::vector<TrajectoryMeta> list() const override;

    /// 设置数据目录（保存到磁盘时使用）。空目录 = 纯内存模式。
    void set_data_dir(const std::string& dir);

    /**
     * @brief 把录制目录导入为轨迹资产。
     * @param recording_dir 录制 session 目录（含 states.csv）
     * @param out_id        生成的轨迹 id（"traj_<session>"，同一录制幂等覆盖）
     */
    Result import_recording(const std::string& recording_dir,
                            std::string& out_id);

private:
    /// 写磁盘资产；data_dir_ 为空返回 ok（纯内存）。失败返回错误。
    Result write_assets_locked(const Trajectory& traj);
    /// 从磁盘读 manifest 到 meta；目录或 manifest 缺失返回 false。
    bool read_manifest_locked(const std::string& id,
                              TrajectoryMeta& meta) const;
    /// 从磁盘加载轨迹到 out（含状态点）。缺失返回 false。
    bool load_from_disk_locked(const std::string& id, Trajectory& out) const;
    /// 扫描 data_dir 下全部资产目录的 manifest 元数据。
    std::vector<TrajectoryMeta> scan_disk_locked() const;

    mutable std::mutex mutex_;
    std::unordered_map<std::string, Trajectory> store_;
    std::string data_dir_;
};

}  // namespace robotics::domain
