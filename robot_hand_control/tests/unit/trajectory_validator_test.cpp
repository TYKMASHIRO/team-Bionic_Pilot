#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

#include "robotics/trajectory/TrajectoryValidator.hpp"

using namespace robotics::domain;

namespace {

Trajectory make_trajectory() {
    Trajectory t;
    t.meta.version = "1";
    for (std::int64_t i = 0; i < 3; ++i) {
        TrajectoryPoint p;
        p.t_offset_ns = i * 100'000'000;
        p.arm.valid = true;
        p.arm.joint_state.valid = true;
        p.arm.joint_state.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
        p.hand.valid = true;
        p.hand.position = {255, 128, 255, 0, 64, 200};
        t.points.push_back(p);
    }
    t.meta.duration_s = 0.2;
    return t;
}

}  // namespace

TEST(TrajectoryValidator, ValidTrajectoryPasses) {
    Trajectory traj = make_trajectory();
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    ASSERT_TRUE(report.valid) << report.to_string();
    EXPECT_TRUE(report.monotonic);
    EXPECT_TRUE(report.finite);
    EXPECT_TRUE(report.in_range);
    EXPECT_TRUE(report.dof_ok);
    EXPECT_EQ(report.sample_count, 3u);
}

TEST(TrajectoryValidator, EmptyTrajectoryFails) {
    Trajectory traj;
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
}

TEST(TrajectoryValidator, VersionMismatchFails) {
    Trajectory traj = make_trajectory();
    traj.meta.version = "999";
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
}

TEST(TrajectoryValidator, NonMonotonicTimelineFails) {
    Trajectory traj = make_trajectory();
    traj.points[2].t_offset_ns = traj.points[1].t_offset_ns - 1;
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.monotonic);
}

TEST(TrajectoryValidator, NaNValueFails) {
    Trajectory traj = make_trajectory();
    traj.points[1].arm.joint_state.position[3] =
        std::numeric_limits<double>::quiet_NaN();
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.finite);
}

TEST(TrajectoryValidator, OutOfRangeFails) {
    Trajectory traj = make_trajectory();
    traj.points[1].arm.joint_state.position[0] = 100.0;  // 超出软限位
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.in_range);
}

TEST(TrajectoryValidator, HandOutOfRawRangeFails) {
    Trajectory traj = make_trajectory();
    traj.points[0].hand.position[2] = 999.0;
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    EXPECT_FALSE(report.valid);
    EXPECT_FALSE(report.in_range);
}

TEST(TrajectoryValidator, WarningsForMissingDevice) {
    Trajectory traj = make_trajectory();
    for (auto& p : traj.points) {
        p.hand.valid = false;
    }
    TrajectoryValidationReport report;
    TrajectoryValidator::validate(traj, report);
    ASSERT_TRUE(report.valid);  // 双设备缺失仅警告
    bool hand_warned = false;
    for (const auto& w : report.warnings) {
        if (w.find("灵巧手") != std::string::npos) hand_warned = true;
    }
    EXPECT_TRUE(hand_warned);
}
