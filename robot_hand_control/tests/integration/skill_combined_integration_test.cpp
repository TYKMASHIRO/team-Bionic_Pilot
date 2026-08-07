#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
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

namespace {

// 写一个最小可用 manifest（load_skills 需要目录下 *.yaml）。
void write_manifest(const std::string& dir, const std::string& id,
                    const std::string& resources, bool real_motion = true) {
    std::ofstream ofs(dir + "/" + id + ".yaml");
    ofs << "id: " << id << "\n"
        << "version: \"1.0.0\"\n"
        << "description: \"test\"\n"
        << "required_resources: [" << resources << "]\n"
        << "safety_profile:\n"
        << "  real_motion: " << (real_motion ? "true" : "false") << "\n";
    if (id == "arm.move_to_safe_pose" || id == "combined.safe_release") {
        ofs << "safe_pose: [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]\n"
            << "safe_pose_speed: 0.3\n";
    } else if (id == "hand.open" || id == "hand.close") {
        ofs << "preset: \"" << (id == "hand.open" ? "open" : "close")
            << "\"\n";
    }
    ofs.close();
}

}  // namespace

// 全链路：加载 manifest → 录制 → 导入轨迹 → 同步复现 → 安全释放。
TEST(IntegrationSkillCombined, RecordImportReplaySafeRelease) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);
    ASSERT_TRUE(app.connect_all().success);

    // 1. 构造 skills 目录（TempDir 固定，先清理）
    const std::string skills_dir = ::testing::TempDir() + "/skills_it";
    fs::remove_all(skills_dir);
    fs::create_directories(skills_dir);
    write_manifest(skills_dir, "hand.open", "\"hand\"");
    write_manifest(skills_dir, "hand.close", "\"hand\"");
    write_manifest(skills_dir, "hand.apply_preset", "\"hand\"");
    write_manifest(skills_dir, "arm.move_to_safe_pose", "\"arm\"");
    write_manifest(skills_dir, "combined.safe_release", "\"arm\", \"hand\"");
    write_manifest(skills_dir, "combined.synchronized_replay",
                  "\"arm\", \"hand\"");
    write_manifest(skills_dir, "arm.drag_teach_record", "\"arm\", \"hand\"");

    ASSERT_TRUE(app.load_skills(skills_dir).success);
    ASSERT_EQ(app.skill_list().size(), 7u);

    // 2. 录制一段 session
    ASSERT_TRUE(app.start_collection().success);
    const std::string rec_dir = ::testing::TempDir() + "/skill_rec";
    fs::remove_all(rec_dir);
    ASSERT_TRUE(app.record_start(rec_dir, 100.0, "deadbeef").success);
    ASSERT_TRUE(app.record_event("teach-start").success);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ASSERT_TRUE(app.record_stop().success);
    ASSERT_TRUE(app.stop_collection().success);
    const std::string session = app.recording_metadata().session_id;

    // 3. 导入为轨迹
    const std::string repo_dir = rec_dir + "/traj_repo";
    fs::remove_all(repo_dir);
    app.set_trajectory_data_dir(repo_dir);
    std::string traj_id;
    ASSERT_TRUE(app.trajectory_import(rec_dir + "/" + session, traj_id).success);
    EXPECT_FALSE(traj_id.empty());

    // 4. 未启用运动时，真实 skill 应被安全门拒绝
    {
        auto r = app.run_skill("hand.open", "{}", /*dry_run=*/false);
        EXPECT_FALSE(r.success);
        EXPECT_EQ(r.final_state, domain::CommandState::SafetyStopped);
        EXPECT_TRUE(r.safety_stopped);
    }

    // 5. 启用运动后，真实执行各 skill
    app.enable_real_motion();
    {
        auto r = app.run_skill("hand.open", "{}", /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
        EXPECT_EQ(r.final_state, domain::CommandState::Succeeded);
    }
    {
        auto r = app.run_skill("hand.close", "{}", /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
    }
    {
        auto r = app.run_skill("hand.apply_preset", R"({"preset":"open"})",
                               /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
    }
    {
        auto r = app.run_skill("arm.move_to_safe_pose", "{}",
                               /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
    }

    // 6. 同步复现刚录制的轨迹
    {
        auto r = app.run_skill(
            "combined.synchronized_replay",
            R"({"trajectory_id":")" + traj_id + R"(","timeout_ms":8000})",
            /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
        EXPECT_FALSE(r.trajectory_version.empty());
    }

    // 7. 安全释放（手张开 + 臂安全位）
    {
        auto r = app.run_skill("combined.safe_release", "{}",
                               /*dry_run=*/false);
        EXPECT_TRUE(r.success) << r.error.to_string();
        const auto hst = hand->get_state();
        for (std::size_t j = 0; j < 6u; ++j) {
            EXPECT_EQ(hst.position[j], 255) << "手应完全张开";
        }
    }

    // 8. 未注册 skill -> lookup 失败
    {
        auto r = app.run_skill("nope.not_exist", "{}", /*dry_run=*/true);
        EXPECT_FALSE(r.success);
        EXPECT_EQ(r.failed_stage, "lookup");
    }

    app.disconnect_all();
}

// dry-run：只校验不运动；设备不连接也能通过（在线检查除外）。
TEST(IntegrationSkillCombined, DryRunDoesNotMove) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);
    ASSERT_TRUE(app.connect_all().success);

    const std::string skills_dir = ::testing::TempDir() + "/skills_dry";
    fs::remove_all(skills_dir);
    fs::create_directories(skills_dir);
    write_manifest(skills_dir, "hand.open", "\"hand\"");
    ASSERT_TRUE(app.load_skills(skills_dir).success);

    // dry-run 返回成功但不执行（设备未 enable_motion 也能过）
    auto r = app.run_skill("hand.open", "{}", /*dry_run=*/true);
    EXPECT_TRUE(r.success);
    EXPECT_EQ(r.failed_stage, "dry_run");

    app.disconnect_all();
}
