#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <string>
#include <thread>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/orchestration/ApplicationService.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics;

namespace fs = std::filesystem;

// 集成链路：采集+录制 → 导入轨迹 → 校验 → 复现（dry-run 与真实）
TEST(IntegrationTrajectoryReplay, RecordImportValidateReplay) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);

    // 1. 录制一段含状态的 session
    ASSERT_TRUE(app.connect_all().success);
    ASSERT_TRUE(app.start_collection().success);
    const std::string rec_dir = ::testing::TempDir();
    ASSERT_TRUE(app.record_start(rec_dir, 100.0, "deadbeef").success);
    ASSERT_TRUE(app.record_event("teach-start").success);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ASSERT_TRUE(app.record_stop().success);
    ASSERT_TRUE(app.stop_collection().success);

    const std::string session = app.recording_metadata().session_id;
    const std::string recording_path = rec_dir + "/" + session;

    // 2. 导入为轨迹（内存 + 磁盘双写）
    const std::string repo_dir = rec_dir + "/traj_repo";
    fs::remove_all(repo_dir);  // 清理残留（TempDir 固定 /tmp/）
    app.set_trajectory_data_dir(repo_dir);
    std::string traj_id;
    ASSERT_TRUE(app.trajectory_import(recording_path, traj_id).success);
    EXPECT_FALSE(traj_id.empty());

    // 3. 列表/加载
    const auto metas = app.trajectory_list();
    ASSERT_EQ(metas.size(), 1u);
    EXPECT_EQ(metas[0].trajectory_id, traj_id);
    EXPECT_EQ(metas[0].source_recording, session);

    domain::Trajectory traj;
    ASSERT_TRUE(app.trajectory_load(traj_id, traj).success);
    EXPECT_GT(traj.points.size(), 0u);
    EXPECT_TRUE(traj.points.front().arm.valid);
    EXPECT_TRUE(traj.points.front().hand.valid);

    // 4. 校验通过
    domain::TrajectoryValidationReport vr;
    ASSERT_TRUE(app.trajectory_validate(traj_id, vr).success);
    EXPECT_TRUE(vr.valid) << vr.to_string();

    // 5. dry-run 复现（不运动）
    domain::ReplayOptions opts;
    opts.speed = 1000.0;
    domain::ReplayReport rep;
    ASSERT_TRUE(app.trajectory_replay(traj_id, opts, /*dry_run=*/true,
                                      {}, rep).success);
    EXPECT_TRUE(rep.dry_run);
    EXPECT_TRUE(rep.success);

    // 6. 真实复现：未启用运动应拒绝
    domain::ReplayReport rep2;
    auto r = app.trajectory_replay(traj_id, opts, /*dry_run=*/false, {}, rep2);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error.category, domain::ErrorCategory::Safety);

    // 7. 启用运动后真实复现成功
    app.enable_real_motion();
    domain::ReplayReport rep3;
    ASSERT_TRUE(app.trajectory_replay(traj_id, opts, /*dry_run=*/false, {},
                                      rep3).success);
    EXPECT_TRUE(rep3.success) << rep3.error.to_string();
    EXPECT_GT(rep3.points_played, 0u);
    EXPECT_EQ(rep3.total_points, traj.points.size());

    app.disconnect_all();
}

// 跨实例可见性：磁盘写入后，新仓库实例可加载（模拟 CLI 进程隔离）
TEST(IntegrationTrajectoryReplay, RepositoryPersistenceAcrossInstances) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);
    ASSERT_TRUE(app.connect_all().success);
    ASSERT_TRUE(app.start_collection().success);
    const std::string rec_dir = ::testing::TempDir();
    ASSERT_TRUE(app.record_start(rec_dir, 100.0, "cafebabe").success);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    ASSERT_TRUE(app.record_stop().success);
    ASSERT_TRUE(app.stop_collection().success);

    const std::string repo_dir = rec_dir + "/traj_repo2";
    fs::remove_all(repo_dir);  // 清理残留
    app.set_trajectory_data_dir(repo_dir);
    std::string traj_id;
    ASSERT_TRUE(
        app.trajectory_import(rec_dir + "/" + app.recording_metadata().session_id,
                              traj_id).success);

    // 新实例（同一 data_dir），模拟独立进程
    domain::ApplicationService app2(clock, arm, hand, store, safety);
    app2.set_trajectory_data_dir(repo_dir);
    domain::Trajectory loaded;
    ASSERT_TRUE(app2.trajectory_load(traj_id, loaded).success);
    EXPECT_GT(loaded.points.size(), 0u);
    const auto metas2 = app2.trajectory_list();
    ASSERT_EQ(metas2.size(), 1u);
    EXPECT_EQ(metas2[0].trajectory_id, traj_id);

    app.disconnect_all();
}
