#include "src/trajectory/TrajectoryRepository.hpp"

#include <algorithm>

namespace robotics::domain {

void TrajectoryRepository::set_data_dir(const std::string& dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_dir_ = dir;
}

Result TrajectoryRepository::load(const std::string& trajectory_id,
                                  Trajectory& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = store_.find(trajectory_id);
    if (it == store_.end()) {
        return Result::fail(Error::make(
            ErrorCategory::Validation, DeviceType::Combined, "TrajectoryRepository",
            1, "轨迹不存在: " + trajectory_id));
    }
    out = it->second;
    return Result::ok();
}

Result TrajectoryRepository::save(const Trajectory& trajectory,
                                  std::string& out_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    Trajectory copy = trajectory;
    if (copy.meta.trajectory_id.empty()) {
        copy.meta.trajectory_id =
            "traj_" + std::to_string(store_.size() + 1);
    }
    out_id = copy.meta.trajectory_id;
    copy.meta.sample_count = copy.points.size();
    if (!copy.points.empty()) {
        copy.meta.created = copy.points.front().timestamp;
        const auto& last = copy.points.back();
        copy.meta.duration_s =
            static_cast<double>(last.t_offset_ns - copy.points.front().t_offset_ns) / 1e9;
    }
    store_[out_id] = std::move(copy);
    return Result::ok();
}

std::vector<TrajectoryMeta> TrajectoryRepository::list() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TrajectoryMeta> metas;
    metas.reserve(store_.size());
    for (const auto& [id, traj] : store_) {
        metas.push_back(traj.meta);
    }
    std::sort(metas.begin(), metas.end(),
              [](const TrajectoryMeta& a, const TrajectoryMeta& b) {
                  return a.created.steady < b.created.steady;
              });
    return metas;
}

}  // namespace robotics::domain
