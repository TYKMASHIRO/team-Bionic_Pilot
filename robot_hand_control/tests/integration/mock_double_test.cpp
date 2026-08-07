#include <gtest/gtest.h>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/orchestration/ApplicationService.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics;

// 集成测试：Mock 双设备组合
TEST(MockDoubleTest, CombinedWorkflow) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    robotics::infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);

    // 连接双设备
    auto r = app.connect_all();
    ASSERT_TRUE(r.success) << r.error.to_string();

    // 状态中心应包含双设备
    auto st = app.current_state();
    EXPECT_TRUE(st.arm_present());
    EXPECT_TRUE(st.hand_present());

    // 未启用真实运动时，运动应被拒绝
    auto home_res = app.arm_home(false);
    EXPECT_FALSE(home_res.success);
    EXPECT_EQ(home_res.error.category, domain::ErrorCategory::Safety);

    // 启用真实运动后，Mock 可运动
    app.enable_real_motion();
    home_res = app.arm_home(false);
    EXPECT_TRUE(home_res.success) << home_res.error.to_string();

    // 灵巧手预设
    auto open_res = app.hand_open(true);
    EXPECT_TRUE(open_res.success);
    auto close_res = app.hand_close(true);
    EXPECT_TRUE(close_res.success);

    // 断开
    app.disconnect_all();
    EXPECT_FALSE(arm->is_connected());
    EXPECT_FALSE(hand->is_connected());
}

TEST(MockDoubleTest, SingleDeviceOfflineDoesNotCrash) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto store = std::make_shared<domain::StateStore>();
    robotics::infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    // 只有机械臂，无灵巧手
    domain::ApplicationService app(clock, arm, nullptr, store, safety);

    auto r = app.arm_connect();
    EXPECT_TRUE(r.success) << r.error.to_string();

    auto st = app.current_state();
    EXPECT_TRUE(st.arm_present());
    EXPECT_FALSE(st.hand_present());

    // 灵巧手操作应优雅失败而非崩溃
    auto res = app.hand_open(true);
    EXPECT_FALSE(res.success);
}

TEST(MockDoubleTest, DragTeachReplayMock) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto store = std::make_shared<domain::StateStore>();
    robotics::infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);
    domain::ApplicationService app(clock, arm, nullptr, store, safety);

    ASSERT_TRUE(app.arm_connect().success);

    // 无轨迹时复现应失败
    auto replay = arm->replay_trajectory(true);
    EXPECT_FALSE(replay.success);

    // 记录拖动示教后复现
    ASSERT_TRUE(app.arm_drag_teach_start(true).success);
    ASSERT_TRUE(app.arm_drag_teach_stop().success);
    EXPECT_TRUE(arm->replay_trajectory(true).success);
}
