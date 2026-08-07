#include <gtest/gtest.h>

#include "robotics/safety/SafetySupervisor.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics::domain;

// 验收：安全拒绝运动测试
TEST(SafetyRejectTest, MotionRejectedWhenNotEnabled) {
    robotics::infra::SafetyConfig cfg;
    SafetySupervisor supervisor(cfg);

    EXPECT_FALSE(supervisor.motion_allowed());
    EXPECT_FALSE(supervisor.real_motion_enabled());
}

TEST(SafetyRejectTest, MotionAllowedAfterExplicitEnable) {
    robotics::infra::SafetyConfig cfg;
    SafetySupervisor supervisor(cfg);
    supervisor.set_real_motion_enabled(true);

    EXPECT_TRUE(supervisor.motion_allowed());
    EXPECT_TRUE(supervisor.real_motion_enabled());
}

TEST(SafetyRejectTest, StaleStateRejected) {
    robotics::infra::SafetyConfig cfg;
    cfg.max_state_delay_ms = 10.0;  // 严格阈值
    SafetySupervisor supervisor(cfg);
    supervisor.set_real_motion_enabled(true);

    CombinedRobotState st;
    st.arm.valid = true;
    // 构造过期时间戳（100ms 前）
    st.arm.timestamp.steady = std::chrono::steady_clock::now() -
                              std::chrono::milliseconds(100);

    auto e = supervisor.evaluate(st);
    EXPECT_TRUE(e.is_error());
    EXPECT_EQ(e.category, ErrorCategory::StateStale);
}

TEST(SafetyRejectTest, ForceLimitExceeded) {
    robotics::infra::SafetyConfig cfg;
    cfg.max_force_n = 50.0;
    SafetySupervisor supervisor(cfg);
    supervisor.set_real_motion_enabled(true);

    CombinedRobotState st;
    st.arm.valid = true;
    st.arm.timestamp.steady = std::chrono::steady_clock::now();
    st.arm.force_torque.force[0] = 100.0;  // 超过 50N

    auto e = supervisor.evaluate(st);
    EXPECT_TRUE(e.is_error());
    EXPECT_EQ(e.category, ErrorCategory::Safety);
}

TEST(SafetyRejectTest, HandTemperatureExceeded) {
    robotics::infra::SafetyConfig cfg;
    cfg.max_hand_temperature_c = 60.0;
    SafetySupervisor supervisor(cfg);
    supervisor.set_real_motion_enabled(true);

    CombinedRobotState st;
    st.hand.valid = true;
    st.hand.timestamp.steady = std::chrono::steady_clock::now();
    st.hand.temperature.fill(85.0);

    auto e = supervisor.evaluate(st);
    EXPECT_TRUE(e.is_error());
    EXPECT_EQ(e.category, ErrorCategory::Safety);
}

TEST(SafetyRejectTest, EmergencyStopDisablesMotion) {
    robotics::infra::SafetyConfig cfg;
    SafetySupervisor supervisor(cfg);
    supervisor.set_real_motion_enabled(true);
    EXPECT_TRUE(supervisor.motion_allowed());

    supervisor.request_stop(StopLevel::EmergencyStop);
    EXPECT_FALSE(supervisor.motion_allowed());
    EXPECT_FALSE(supervisor.real_motion_enabled());
}
