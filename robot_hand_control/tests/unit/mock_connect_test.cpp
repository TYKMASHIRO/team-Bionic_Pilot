#include <gtest/gtest.h>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"

using namespace robotics::domain;
using robotics::mock::MockDexterousHand;
using robotics::mock::MockRobotArm;

// 验收：Mock 连接测试
TEST(MockConnectTest, RobotArmConnectDisconnect) {
    MockRobotArm arm;
    EXPECT_FALSE(arm.is_connected());

    auto r = arm.connect();
    EXPECT_TRUE(r.success) << r.error.to_string();
    EXPECT_TRUE(arm.is_connected());

    arm.disconnect();
    EXPECT_FALSE(arm.is_connected());
}

TEST(MockConnectTest, DexterousHandConnectDisconnect) {
    MockDexterousHand hand;
    EXPECT_FALSE(hand.is_connected());

    auto r = hand.connect();
    EXPECT_TRUE(r.success) << r.error.to_string();
    EXPECT_TRUE(hand.is_connected());

    hand.disconnect();
    EXPECT_FALSE(hand.is_connected());
}

TEST(MockConnectTest, ArmNotConnectedRejectsMotion) {
    MockRobotArm arm;
    ArmJointVector joint{};
    auto r = arm.move_joint(joint, 0.2, true);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error.category, ErrorCategory::NotConnected);
}

TEST(MockConnectTest, HandNotConnectedRejectsMotion) {
    MockDexterousHand hand;
    HandJointVector pos{};
    auto r = hand.set_joint_positions(pos);
    EXPECT_FALSE(r.success);
    EXPECT_EQ(r.error.category, ErrorCategory::NotConnected);
}
