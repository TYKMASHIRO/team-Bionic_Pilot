#include "src/trajectory/TrajectoryRepository.hpp"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

#include "robotics/services/RecordingMetadata.hpp"
#include "src/domain/recording/CsvStateColumns.hpp"
#include "src/trajectory/TrajectoryCsvLoader.hpp"

namespace robotics::domain {

namespace {
namespace fs = std::filesystem;

constexpr const char* kManifestName = "manifest.txt";
constexpr const char* kStatesName = "states.csv";
constexpr const char* kEventsName = "events.csv";
constexpr const char* kMetadataName = "metadata.txt";

/// key=value 行读取。
std::string meta_value(const std::string& text, const std::string& key) {
    const std::string needle = key + "=";
    const std::size_t pos = text.find(needle);
    if (pos == std::string::npos) return {};
    const std::size_t val = pos + needle.size();
    const std::size_t end = text.find('\n', val);
    return text.substr(val, end == std::string::npos ? std::string::npos
                                                     : end - val);
}

std::string read_file(const fs::path& p) {
    std::ifstream f(p);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

Result disk_error(const std::string& what) {
    return Result::fail(Error::make(
        ErrorCategory::Internal, DeviceType::Combined, "TrajectoryRepository",
        10, "轨迹磁盘操作失败: " + what));
}

/// 从 TrajectoryPoint 组装 CombinedRobotState（写 states.csv 用）。
CombinedRobotState point_to_state(const TrajectoryPoint& p) {
    CombinedRobotState c;
    c.timestamp = p.timestamp;
    c.arm = p.arm;
    c.hand = p.hand;
    return c;
}

}  // namespace

void TrajectoryRepository::set_data_dir(const std::string& dir) {
    std::lock_guard<std::mutex> lock(mutex_);
    data_dir_ = dir;
}

// ---------------------------------------------------------------------------
// 加载
// ---------------------------------------------------------------------------
Result TrajectoryRepository::load(const std::string& trajectory_id,
                                  Trajectory& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = store_.find(trajectory_id);
    if (it != store_.end()) {
        out = it->second;
        return Result::ok();
    }
    // 内存未命中 -> 磁盘
    if (!data_dir_.empty()) {
        Trajectory traj;
        if (load_from_disk_locked(trajectory_id, traj)) {
            store_[trajectory_id] = traj;
            out = std::move(traj);
            return Result::ok();
        }
    }
    return Result::fail(Error::make(
        ErrorCategory::Validation, DeviceType::Combined, "TrajectoryRepository",
        1, "轨迹不存在: " + trajectory_id));
}

// ---------------------------------------------------------------------------
// 保存
// ---------------------------------------------------------------------------
Result TrajectoryRepository::save(const Trajectory& trajectory,
                                  std::string& out_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    Trajectory copy = trajectory;
    if (copy.meta.trajectory_id.empty()) {
        // 未指定 id：基于磁盘现有资产数 + 内存缓存生成不冲突的新序号
        std::size_t n = 0;
        if (!data_dir_.empty()) {
            std::error_code ec;
            for (const auto& entry : fs::directory_iterator(data_dir_, ec)) {
                const std::string name = entry.path().filename().string();
                if (name.rfind("traj_", 0) == 0) {
                    const std::size_t num =
                        std::strtoull(name.c_str() + 5, nullptr, 10);
                    n = std::max(n, num);
                }
            }
        }
        while (store_.count("traj_" + std::to_string(n + 1)) > 0) ++n;
        copy.meta.trajectory_id = "traj_" + std::to_string(n + 1);
    }
    out_id = copy.meta.trajectory_id;
    copy.meta.sample_count = copy.points.size();
    if (!copy.points.empty()) {
        copy.meta.created = copy.points.front().timestamp;
        const auto& last = copy.points.back();
        copy.meta.duration_s =
            static_cast<double>(last.t_offset_ns - copy.points.front().t_offset_ns) / 1e9;
    }

    if (!data_dir_.empty()) {
        const Result wr = write_assets_locked(copy);
        if (!wr.success) return wr;
    }
    store_[out_id] = std::move(copy);
    return Result::ok();
}

// ---------------------------------------------------------------------------
// 导入录制
// ---------------------------------------------------------------------------
Result TrajectoryRepository::import_recording(const std::string& recording_dir,
                                              std::string& out_id) {
    Trajectory traj;
    const Result ld = TrajectoryCsvLoader::load_recording(recording_dir, traj);
    if (!ld.success) return ld;

    const std::string session = fs::path(recording_dir).filename().string();
    traj.meta.trajectory_id = "traj_" + session;
    return save(traj, out_id);
}

// ---------------------------------------------------------------------------
// 列出
// ---------------------------------------------------------------------------
std::vector<TrajectoryMeta> TrajectoryRepository::list() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TrajectoryMeta> metas;
    metas.reserve(store_.size());

    // 磁盘资产（先），避免被内存缓存漏掉
    for (const auto& m : scan_disk_locked()) metas.push_back(m);
    for (const auto& [id, traj] : store_) metas.push_back(traj.meta);

    // 去重（磁盘与内存同 id 时以内存为准）
    std::sort(metas.begin(), metas.end(),
              [](const TrajectoryMeta& a, const TrajectoryMeta& b) {
                  if (a.trajectory_id == b.trajectory_id) {
                      return a.created.steady > b.created.steady;  // 后写的优先
                  }
                  return a.trajectory_id < b.trajectory_id;
              });
    auto last = std::unique(metas.begin(), metas.end(),
                            [](const TrajectoryMeta& a, const TrajectoryMeta& b) {
                                return a.trajectory_id == b.trajectory_id;
                            });
    metas.erase(last, metas.end());

    std::sort(metas.begin(), metas.end(),
              [](const TrajectoryMeta& a, const TrajectoryMeta& b) {
                  return a.created.steady < b.created.steady;
              });
    return metas;
}

// ---------------------------------------------------------------------------
// 磁盘层
// ---------------------------------------------------------------------------
Result TrajectoryRepository::write_assets_locked(const Trajectory& traj) {
    const fs::path dir = fs::path(data_dir_) / traj.meta.trajectory_id;
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) return disk_error("创建目录失败: " + dir.string());

    // states.csv
    {
        std::ofstream out(dir / kStatesName,
                          std::ios::out | std::ios::trunc);
        if (!out) return disk_error("打开 states.csv 失败: " +
                                    (dir / kStatesName).string());
        CombinedRobotState dummy;
        if (!CsvStateColumns::write_row(out, dummy, /*header=*/true)) {
            return disk_error("写 states.csv 表头失败");
        }
        out << '\n';
        for (const auto& p : traj.points) {
            if (!CsvStateColumns::write_row(out, point_to_state(p),
                                            /*header=*/false)) {
                return disk_error("写 states.csv 状态行失败");
            }
            out << '\n';
        }
    }

    // events.csv（轨迹点内嵌事件）
    {
        std::ofstream out(dir / kEventsName, std::ios::out | std::ios::trunc);
        if (!out) return disk_error("打开 events.csv 失败");
        for (const auto& p : traj.points) {
            if (p.event.empty()) continue;
            out << p.timestamp.to_iso8601() << ',' << p.timestamp.wall_ns()
                << ',' << p.timestamp.steady_ns() << ','
                << csv_escape_shared(p.event) << '\n';
        }
    }

    // metadata.txt（供 TrajectoryCsvLoader 校验版本/取哈希）
    {
        std::ofstream out(dir / kMetadataName, std::ios::out | std::ios::trunc);
        if (!out) return disk_error("打开 metadata.txt 失败");
        out << "format_version=" << kRecordingFormatVersion << '\n'
            << "config_hash=" << traj.meta.config_hash << '\n'
            << "calibration_ref=" << traj.meta.calibration_id << '\n';
    }

    // manifest.txt（轨迹专属字段）
    {
        std::ofstream out(dir / kManifestName, std::ios::out | std::ios::trunc);
        if (!out) return disk_error("打开 manifest.txt 失败");
        out << "trajectory_id=" << traj.meta.trajectory_id << '\n'
            << "name=" << traj.meta.name << '\n'
            << "source_recording=" << traj.meta.source_recording << '\n'
            << "calibration_id=" << traj.meta.calibration_id << '\n'
            << "config_hash=" << traj.meta.config_hash << '\n'
            << "version=" << traj.meta.version << '\n'
            << "sample_count=" << traj.meta.sample_count << '\n'
            << "duration_s=" << traj.meta.duration_s << '\n'
            << "created_wall_ns=" << traj.meta.created.wall_ns() << '\n'
            << "created_steady_ns=" << traj.meta.created.steady_ns() << '\n';
    }
    return Result::ok();
}

bool TrajectoryRepository::read_manifest_locked(const std::string& id,
                                                TrajectoryMeta& meta) const {
    if (data_dir_.empty()) return false;
    const fs::path dir = fs::path(data_dir_) / id;
    const fs::path mp = dir / kManifestName;
    if (!fs::exists(mp)) return false;
    const std::string text = read_file(mp);
    if (text.empty()) return false;
    meta.trajectory_id = meta_value(text, "trajectory_id");
    meta.name = meta_value(text, "name");
    meta.source_recording = meta_value(text, "source_recording");
    meta.calibration_id = meta_value(text, "calibration_id");
    meta.config_hash = meta_value(text, "config_hash");
    meta.version = meta_value(text, "version");
    meta.sample_count = std::strtoull(
        meta_value(text, "sample_count").c_str(), nullptr, 10);
    meta.duration_s = std::strtod(
        meta_value(text, "duration_s").c_str(), nullptr);
    meta.created.wall =
        std::chrono::system_clock::time_point(std::chrono::nanoseconds(
            std::strtoll(meta_value(text, "created_wall_ns").c_str(), nullptr, 10)));
    meta.created.steady =
        std::chrono::steady_clock::time_point(std::chrono::nanoseconds(
            std::strtoll(meta_value(text, "created_steady_ns").c_str(), nullptr, 10)));
    return true;
}

bool TrajectoryRepository::load_from_disk_locked(const std::string& id,
                                                 Trajectory& out) const {
    if (data_dir_.empty()) return false;
    const fs::path dir = fs::path(data_dir_) / id;
    if (!fs::exists(dir / kStatesName)) return false;

    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir.string(), traj);
    if (!r.success) return false;

    TrajectoryMeta meta;
    if (read_manifest_locked(id, meta)) {
        // manifest 字段优先（source_recording/calibration 等以导入时为准）
        traj.meta.name = meta.name;
        traj.meta.source_recording = meta.source_recording;
        traj.meta.calibration_id = meta.calibration_id;
        traj.meta.config_hash = meta.config_hash;
        traj.meta.version = meta.version;
        traj.meta.created = meta.created;
        traj.meta.sample_count = meta.sample_count;
        traj.meta.duration_s = meta.duration_s;
    }
    traj.meta.trajectory_id = id;
    out = std::move(traj);
    return true;
}

std::vector<TrajectoryMeta> TrajectoryRepository::scan_disk_locked() const {
    std::vector<TrajectoryMeta> metas;
    if (data_dir_.empty()) return metas;
    std::error_code ec;
    if (!fs::exists(data_dir_, ec)) return metas;
    for (const auto& entry : fs::directory_iterator(data_dir_, ec)) {
        if (!entry.is_directory(ec)) continue;
        TrajectoryMeta meta;
        if (read_manifest_locked(entry.path().filename().string(), meta) &&
            !meta.trajectory_id.empty()) {
            metas.push_back(std::move(meta));
        }
    }
    return metas;
}

}  // namespace robotics::domain
