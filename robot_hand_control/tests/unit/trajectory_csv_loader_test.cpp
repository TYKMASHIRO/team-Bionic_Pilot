#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robotics/services/RecordingMetadata.hpp"
#include "src/domain/recording/CsvStateColumns.hpp"
#include "src/trajectory/TrajectoryCsvLoader.hpp"
#include "src/trajectory/TrajectoryRepository.hpp"

using namespace robotics::domain;

namespace {

namespace fs = std::filesystem;

CombinedRobotState make_state(std::int64_t steady_ns) {
    CombinedRobotState c;
    c.timestamp.steady = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(steady_ns));
    c.timestamp.wall = std::chrono::system_clock::now();
    c.arm.valid = true;
    c.arm.fresh = true;
    c.arm.sequence = static_cast<std::uint64_t>(steady_ns);
    c.arm.timestamp = c.timestamp;
    c.arm.joint_state.valid = true;
    c.arm.joint_state.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    c.hand.valid = true;
    c.hand.fresh = true;
    c.hand.sequence = static_cast<std::uint64_t>(steady_ns);
    c.hand.timestamp = c.timestamp;
    c.hand.position = {255, 128, 255, 0, 64, 200};
    return c;
}

/// 写一个最小录制目录，返回目录路径。
std::string write_recording(const std::string& name,
                            const std::vector<std::int64_t>& steady_ns,
                            const std::string& format_version = "1",
                            bool with_events = false) {
    const std::string dir =
        std::string(::testing::TempDir()) + "/" + name;
    fs::remove_all(dir);
    fs::create_directories(dir);

    {
        std::ofstream md(dir + "/metadata.txt");
        md << "format_version=" << format_version << "\n"
           << "config_hash=hashABC\n"
           << "calibration_ref=calib1\n";
    }
    {
        std::ofstream st(dir + "/states.csv");
        CombinedRobotState dummy;
        CsvStateColumns::write_row(st, dummy, true);
        st << '\n';
        for (const auto ns : steady_ns) {
            CsvStateColumns::write_row(st, make_state(ns), false);
            st << '\n';
        }
    }
    if (with_events) {
        std::ofstream ev(dir + "/events.csv");
        // iso, wall_ns, steady_ns, event
        // 点时间轴: 100/200/300ns。事件 1 在 100-200 之间（挂到点 t_offset=100），
        // 事件 2 在 200-300 之间（挂到点 t_offset=200）。
        ev << "2026-08-07T00:00:00,0,"
           << (steady_ns.empty() ? 0 : steady_ns.front() + 50)
           << ",teach-start\n";
        ev << "2026-08-07T00:00:01,0,"
           << (steady_ns.empty() ? 0 : steady_ns.front() + 150)
           << ",grasp\n";
    }
    return dir;
}

}  // namespace

TEST(TrajectoryCsvLoader, LoadsValidRecording) {
    const std::string dir =
        write_recording("rec_load_valid", {100, 200, 300});
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_TRUE(r.success) << r.error.to_string();

    EXPECT_EQ(traj.points.size(), 3u);
    EXPECT_EQ(traj.points[0].t_offset_ns, 0);
    EXPECT_EQ(traj.points[1].t_offset_ns, 100);
    EXPECT_EQ(traj.points[2].t_offset_ns, 200);

    // meta 字段
    EXPECT_EQ(traj.meta.source_recording, "rec_load_valid");
    EXPECT_EQ(traj.meta.version, "1");
    EXPECT_EQ(traj.meta.sample_count, 3u);
    EXPECT_EQ(traj.meta.config_hash, "hashABC");
    EXPECT_EQ(traj.meta.calibration_id, "calib1");
    EXPECT_NEAR(traj.meta.duration_s, 200.0 / 1e9, 1e-6);

    // 状态在场
    EXPECT_TRUE(traj.points[0].arm.valid);
    EXPECT_TRUE(traj.points[0].hand.valid);
}

TEST(TrajectoryCsvLoader, AttachesEventsToNearestPoint) {
    const std::string dir =
        write_recording("rec_load_events", {100, 200, 300}, "1",
                        /*with_events=*/true);
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_TRUE(r.success) << r.error.to_string();

    // 事件 steady=115 挂到点 t_offset>=15 的第一个点（t_offset=100）；
    // 事件 steady=285 挂到点 t_offset>=185 的第一个点（t_offset=200）。
    EXPECT_EQ(traj.points[1].event, "teach-start");
    EXPECT_EQ(traj.points[2].event, "grasp");
}

TEST(TrajectoryCsvLoader, RejectsFormatVersionMismatch) {
    const std::string dir =
        write_recording("rec_load_badver", {100, 200}, "999");
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_FALSE(r.success);
    EXPECT_EQ(r.error.category, ErrorCategory::Validation);
}

TEST(TrajectoryCsvLoader, RejectsMissingStates) {
    const std::string dir =
        std::string(::testing::TempDir()) + "/rec_missing_states";
    fs::remove_all(dir);
    fs::create_directories(dir);
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_FALSE(r.success);
}

TEST(TrajectoryCsvLoader, RejectsEmptyRecording) {
    const std::string dir = write_recording("rec_load_empty", {});
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_FALSE(r.success);
}

TEST(TrajectoryCsvLoader, RejectsMalformedRow) {
    const std::string dir = write_recording("rec_load_mal", {100});
    // 追加一行坏数据（列数不足）
    {
        std::ofstream st(dir + "/states.csv", std::ios::app);
        st << "1,2,3\n";
    }
    Trajectory traj;
    const Result r = TrajectoryCsvLoader::load_recording(dir, traj);
    ASSERT_FALSE(r.success);
    EXPECT_EQ(r.error.category, ErrorCategory::Validation);
}

// 往返：仓库 save 写资产 -> 另一个仓库 load -> 字段一致（磁盘持久化核心链路）
TEST(TrajectoryCsvLoader, RoundTripThroughRepositoryAssets) {
    const std::string dir =
        write_recording("rec_roundtrip", {500, 600, 700, 800}, "1");
    Trajectory loaded;
    ASSERT_TRUE(TrajectoryCsvLoader::load_recording(dir, loaded).success);

    const std::string asset_dir =
        std::string(::testing::TempDir()) + "/traj_roundtrip_assets";
    fs::remove_all(asset_dir);

    // 仓库 A：写资产（save 带 id）
    TrajectoryRepository repo_a;
    repo_a.set_data_dir(asset_dir);
    std::string id;
    ASSERT_TRUE(repo_a.save(loaded, id).success);

    // 仓库 B（新实例，模拟跨进程）：按 id 从磁盘加载
    TrajectoryRepository repo_b;
    repo_b.set_data_dir(asset_dir);
    Trajectory reread;
    const Result r = repo_b.load(id, reread);
    ASSERT_TRUE(r.success) << r.error.to_string();
    EXPECT_EQ(reread.points.size(), loaded.points.size());
    EXPECT_EQ(reread.points.front().t_offset_ns, 0);
    for (std::size_t i = 0; i < loaded.points.size(); ++i) {
        EXPECT_EQ(reread.points[i].t_offset_ns, loaded.points[i].t_offset_ns);
        EXPECT_EQ(reread.points[i].arm.valid, loaded.points[i].arm.valid);
        EXPECT_EQ(reread.points[i].hand.valid, loaded.points[i].hand.valid);
    }
}
