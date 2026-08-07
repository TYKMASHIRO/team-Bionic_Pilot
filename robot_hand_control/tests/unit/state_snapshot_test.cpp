#include <gtest/gtest.h>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics::domain;
using robotics::mock::MockDexterousHand;
using robotics::mock::MockRobotArm;

// 验收：状态快照测试
TEST(StateSnapshotTest, ArmStateValidAfterConnect) {
    MockRobotArm arm;
    arm.connect();

    auto st = arm.get_state();
    EXPECT_TRUE(st.valid);
    EXPECT_TRUE(st.fresh);
    EXPECT_EQ(st.sequence, 1);
}

TEST(StateSnapshotTest, HandStateValidAfterConnect) {
    MockDexterousHand hand;
    hand.connect();

    auto st = hand.get_state();
    EXPECT_TRUE(st.valid);
    // 默认张开
    for (std::size_t i = 0; i < kHandDof; ++i) {
        EXPECT_DOUBLE_EQ(st.position[i], 255.0);
    }
}

TEST(StateSnapshotTest, StateStoreCombined) {
    MockRobotArm arm;
    MockDexterousHand hand;
    arm.connect();
    hand.connect();

    StateStore store;
    store.update_arm(arm.get_state());
    store.update_hand(hand.get_state());

    auto c = store.combined();
    EXPECT_TRUE(c.arm_present());
    EXPECT_TRUE(c.hand_present());
    EXPECT_EQ(c.sync_quality, SyncQuality::Good);
    EXPECT_LE(c.time_delta_ns, 50'000'000);
}

TEST(StateSnapshotTest, StateStoreSingleDevice) {
    MockRobotArm arm;
    arm.connect();
    StateStore store;
    store.update_arm(arm.get_state());

    auto c = store.combined();
    EXPECT_TRUE(c.arm_present());
    EXPECT_FALSE(c.hand_present());
}
