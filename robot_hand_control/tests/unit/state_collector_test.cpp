#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/services/StateCollector.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics::domain;
using robotics::infra::SystemClock;
using robotics::mock::MockDexterousHand;
using robotics::mock::MockRobotArm;

// 验收：双设备采集线程连续写入 StateStore，受控停止不崩溃
TEST(StateCollectorTest, DualDeviceContinuousSampling) {
    auto clock = std::make_shared<SystemClock>();
    auto arm = std::make_shared<MockRobotArm>();
    auto hand = std::make_shared<MockDexterousHand>();
    auto store = std::make_shared<StateStore>();
    ASSERT_TRUE(arm->connect().success);
    ASSERT_TRUE(hand->connect().success);

    StateCollector collector(arm, hand, store, clock, 200.0, 200.0);
    ASSERT_TRUE(collector.start().success);
    EXPECT_TRUE(collector.running());

    std::this_thread::sleep_for(std::chrono::milliseconds(150));

    auto latest_arm = store->latest_arm();
    auto latest_hand = store->latest_hand();
    ASSERT_TRUE(latest_arm.has_value());
    ASSERT_TRUE(latest_hand.has_value());
    EXPECT_TRUE(latest_arm->valid);
    EXPECT_TRUE(latest_hand->valid);
    // StateStore 统一打序号：采集后序号非 0 且单调
    EXPECT_GT(latest_arm->sequence, 0);
    EXPECT_GT(latest_hand->sequence, 0);

    ASSERT_TRUE(collector.stop().success);
    EXPECT_FALSE(collector.running());
}

// 单设备离线不崩溃：只传 arm，无 hand
TEST(StateCollectorTest, SingleDeviceNoHandDoesNotCrash) {
    auto clock = std::make_shared<SystemClock>();
    auto arm = std::make_shared<MockRobotArm>();
    auto store = std::make_shared<StateStore>();
    ASSERT_TRUE(arm->connect().success);

    StateCollector collector(arm, nullptr, store, clock, 200.0, 200.0);
    ASSERT_TRUE(collector.start().success);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    auto latest_arm = store->latest_arm();
    ASSERT_TRUE(latest_arm.has_value());
    EXPECT_TRUE(latest_arm->valid);
    EXPECT_FALSE(store->latest_hand().has_value());

    ASSERT_TRUE(collector.stop().success);
    EXPECT_FALSE(collector.running());
}

// 停止后采集线程退出；析构自动 stop
TEST(StateCollectorTest, StopIdempotentAndDestructor) {
    auto clock = std::make_shared<SystemClock>();
    auto arm = std::make_shared<MockRobotArm>();
    auto store = std::make_shared<StateStore>();
    ASSERT_TRUE(arm->connect().success);

    {
        StateCollector collector(arm, nullptr, store, clock, 200.0, 200.0);
        ASSERT_TRUE(collector.start().success);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        // stop 幂等
        ASSERT_TRUE(collector.stop().success);
        ASSERT_TRUE(collector.stop().success);
    }  // 析构自动 stop
    // 停止后 StateStore 快照保留（采集线程退出不销毁数据）
    auto latest = store->latest_arm();
    ASSERT_TRUE(latest.has_value());
    EXPECT_TRUE(latest->valid);
}
