#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <vector>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "robotics/trajectory/TrajectoryReplayer.hpp"
#include "src/infrastructure/time/SystemClock.hpp"

using namespace robotics::domain;
using namespace robotics;

namespace {

infra::SafetyConfig safety_cfg() {
    infra::SafetyConfig cfg;
    cfg.max_state_delay_ms = 5000.0;  // 宽松：测试环境
    return cfg;
}

/// 构造通过校验的轨迹：arm 7 关节 / hand 6 通道，时间轴单调。
Trajectory make_traj(std::size_t n = 3, double step_ns = 100e6) {
    Trajectory t;
    t.meta.version = "1";
    for (std::size_t i = 0; i < n; ++i) {
        TrajectoryPoint p;
        p.t_offset_ns = static_cast<std::int64_t>(i * step_ns);
        p.arm.valid = true;
        p.arm.joint_state.valid = true;
        p.arm.joint_state.position = {
            0.1 + 0.01 * i, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
        p.hand.valid = true;
        p.hand.position = {255, 128, 255, 0, 64, 200};
        t.points.push_back(p);
    }
    t.meta.sample_count = n;
    t.meta.duration_s = (n > 1 ? (n - 1) * step_ns / 1e9 : 0.0);
    return t;
}

}  // namespace

TEST(TrajectoryReplayer, DryRunValidatesWithoutMoving) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    auto clock = std::make_shared<infra::SystemClock>();

    // dry-run 需要设备在线（Prompt.md：dry-run 校验起点偏差/设备在线/安全许可），
    // 但不需要真实运动许可、也不下发任何运动指令。
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);
    TrajectoryReplayer rp(arm, hand, safety, clock);
    ReplayOptions opt;
    opt.dry_run = true;
    ReplayReport rep = rp.replay(make_traj(), opt);
    EXPECT_TRUE(rep.success);
    EXPECT_TRUE(rep.dry_run);
    EXPECT_EQ(rep.total_points, 3u);
    EXPECT_EQ(rep.points_played, 0u);
}

TEST(TrajectoryReplayer, InvalidTrajectoryFailsBeforeMoving) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    auto clock = std::make_shared<infra::SystemClock>();
    TrajectoryReplayer rp(arm, hand, safety, clock);

    Trajectory traj = make_traj();
    traj.points[2].t_offset_ns = traj.points[1].t_offset_ns - 1;  // 非单调
    ReplayReport rep = rp.replay(traj, ReplayOptions{});
    EXPECT_FALSE(rep.success);
    EXPECT_EQ(rep.failed_stage, "validate");
}

TEST(TrajectoryReplayer, RealMotionRequiresConnectionAndSafety) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    auto clock = std::make_shared<infra::SystemClock>();
    TrajectoryReplayer rp(arm, hand, safety, clock);

    // 未连接 -> devices_online
    ReplayReport rep = rp.replay(make_traj(), ReplayOptions{});
    EXPECT_FALSE(rep.success);
    EXPECT_EQ(rep.failed_stage, "devices_online");

    // 连接但未启用真实运动 -> safety
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);
    ReplayReport rep2 = rp.replay(make_traj(), ReplayOptions{});
    EXPECT_FALSE(rep2.success);
    EXPECT_EQ(rep2.failed_stage, "safety");
    EXPECT_TRUE(rep2.safety_stopped);

    // 启用真实运动 -> 播放成功
    safety->set_real_motion_enabled(true);
    ReplayReport rep3 = rp.replay(make_traj(), ReplayOptions{});
    EXPECT_TRUE(rep3.success) << rep3.error.to_string();
    EXPECT_EQ(rep3.failed_stage, "replay");
    EXPECT_EQ(rep3.points_played, 3u);
}

TEST(TrajectoryReplayer, PlaysAllPointsToFinalPose) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    safety->set_real_motion_enabled(true);
    auto clock = std::make_shared<infra::SystemClock>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    TrajectoryReplayer rp(arm, hand, safety, clock);
    const Trajectory traj = make_traj(3);
    ReplayOptions opt;
    opt.speed = 1000.0;
    ReplayReport rep = rp.replay(traj, opt);
    ASSERT_TRUE(rep.success) << rep.error.to_string();

    // arm 应停在末点位置（Mock 立即到位）
    const auto final_arm = arm->get_state();
    const auto& last = traj.points.back();
    for (std::size_t j = 0; j < kArmDof; ++j) {
        EXPECT_DOUBLE_EQ(final_arm.joint_state.position[j],
                         last.arm.joint_state.position[j]);
    }
    // hand 应停在末点位置
    const auto final_hand = hand->get_state();
    for (std::size_t j = 0; j < kHandDof; ++j) {
        EXPECT_DOUBLE_EQ(final_hand.position[j], last.hand.position[j]);
    }
}

TEST(TrajectoryReplayer, CancelStopsEarly) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    safety->set_real_motion_enabled(true);
    auto clock = std::make_shared<infra::SystemClock>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    TrajectoryReplayer rp(arm, hand, safety, clock);
    std::atomic<bool> cancel{true};  // 一开始就取消
    std::function<bool()> cancel_fn = [&]() { return cancel.load(); };
    ReplayOptions opt;
    opt.speed = 1000.0;
    ReplayReport rep = rp.replay(make_traj(5), opt, &cancel_fn);
    EXPECT_TRUE(rep.cancelled);
    EXPECT_FALSE(rep.success);
    EXPECT_LT(rep.points_played, 5u);
}

TEST(TrajectoryReplayer, StartDeviationAlignsThenPlays) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    safety->set_real_motion_enabled(true);
    auto clock = std::make_shared<infra::SystemClock>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    // 制造起点偏差：把 arm 移到与首点差异大的位置
    ArmJointVector far;
    for (std::size_t j = 0; j < kArmDof; ++j) far[j] = 1.5;
    ASSERT_TRUE(arm->move_joint(far, 0.2, true).success);

    TrajectoryReplayer rp(arm, hand, safety, clock);
    const Trajectory traj = make_traj(3);
    ReplayOptions opt;
    opt.speed = 1000.0;
    opt.start_deviation_tol_rad = 0.05;  // 偏差必然超阈值
    ReplayReport rep = rp.replay(traj, opt);
    ASSERT_TRUE(rep.success) << rep.error.to_string();
    EXPECT_GT(rep.start_deviation_rad, 0.05);
    // 播放后回到末点（先对齐首点再播放，末点应一致）
    const auto final_arm = arm->get_state();
    for (std::size_t j = 0; j < kArmDof; ++j) {
        EXPECT_DOUBLE_EQ(final_arm.joint_state.position[j],
                         traj.points.back().arm.joint_state.position[j]);
    }
}

TEST(TrajectoryReplayer, TimeoutStopsReplay) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    safety->set_real_motion_enabled(true);
    auto clock = std::make_shared<infra::SystemClock>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    TrajectoryReplayer rp(arm, hand, safety, clock);
    ReplayOptions opt;
    opt.speed = 1.0;  // 原速：3 点间隔 100ms 共 200ms
    opt.timeout = std::chrono::milliseconds(1);  // 立即超时
    ReplayReport rep = rp.replay(make_traj(3), opt);
    EXPECT_TRUE(rep.timed_out);
    EXPECT_FALSE(rep.success);
    EXPECT_LT(rep.points_played, 3u);
}

TEST(TrajectoryReplayer, StartDeviationRejectsWhenAlignDisabled) {
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto safety = std::make_shared<SafetySupervisor>(safety_cfg());
    safety->set_real_motion_enabled(true);
    auto clock = std::make_shared<infra::SystemClock>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    ArmJointVector far;
    for (std::size_t j = 0; j < kArmDof; ++j) far[j] = 1.5;
    ASSERT_TRUE(arm->move_joint(far, 0.2, true).success);

    TrajectoryReplayer rp(arm, hand, safety, clock);
    ReplayOptions opt;
    opt.align_start = false;
    opt.start_deviation_tol_rad = 0.05;
    ReplayReport rep = rp.replay(make_traj(3), opt);
    EXPECT_FALSE(rep.success);
    EXPECT_EQ(rep.failed_stage, "start_alignment");
}
