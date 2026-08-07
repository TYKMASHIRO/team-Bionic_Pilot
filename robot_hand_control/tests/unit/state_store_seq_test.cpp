#include <gtest/gtest.h>

#include <chrono>

#include "robotics/domain/states/CombinedRobotState.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics::domain;

namespace {

std::chrono::steady_clock::time_point steady_ns(std::int64_t ns) {
    return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
}

}  // namespace

// 统一打序号：latest_*() 携带 update 分配的序号，单调递增
TEST(StateStoreSeqTest, LatestSnapshotsCarryUnifiedSeq) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    DexterousHandState h;
    h.valid = true;
    store.update_arm(a);
    store.update_arm(a);
    store.update_hand(h);

    ASSERT_TRUE(store.latest_arm().has_value());
    ASSERT_TRUE(store.latest_hand().has_value());
    EXPECT_EQ(store.latest_arm()->sequence, 2u);
    EXPECT_EQ(store.latest_hand()->sequence, 1u);
}

// combined() 序号单调递增
TEST(StateStoreSeqTest, CombinedSequenceMonotonic) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    store.update_arm(a);
    const auto c1 = store.combined();
    const auto c2 = store.combined();
    EXPECT_GT(c2.sequence, c1.sequence);
}

// 双设备在场、时间差在阈值内 → Good
TEST(StateStoreSeqTest, GoodWhenBothInSync) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    a.timestamp.steady = steady_ns(1'000'000'000);
    DexterousHandState h;
    h.valid = true;
    h.timestamp.steady = steady_ns(1'000'000'000 + 10'000'000);  // +10ms
    store.update_arm(a);
    store.update_hand(h);

    const auto c = store.combined();
    EXPECT_TRUE(c.arm_present());
    EXPECT_TRUE(c.hand_present());
    EXPECT_EQ(c.sync_quality, SyncQuality::Good);
    EXPECT_EQ(c.time_delta_ns, 10'000'000);
    EXPECT_EQ(c.timestamp.steady_ns(), a.timestamp.steady_ns());  // 取 arm
}

// 时间差超阈值（>50ms）→ Skewed
TEST(StateStoreSeqTest, SkewedWhenDeltaAboveThreshold) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    a.timestamp.steady = steady_ns(0);
    DexterousHandState h;
    h.valid = true;
    h.timestamp.steady = steady_ns(100'000'000);  // +100ms
    store.update_arm(a);
    store.update_hand(h);

    const auto c = store.combined();
    EXPECT_EQ(c.sync_quality, SyncQuality::Skewed);
    EXPECT_EQ(c.time_delta_ns, 100'000'000);
}

// 仅 arm 在场、hand 从未更新 → Lost
TEST(StateStoreSeqTest, ArmOnlyGivesLost) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    store.update_arm(a);

    const auto c = store.combined();
    EXPECT_TRUE(c.arm_present());
    EXPECT_FALSE(c.hand_present());
    EXPECT_EQ(c.sync_quality, SyncQuality::Lost);
}

// 双设备均不在场 → Unknown
TEST(StateStoreSeqTest, NoneGivesUnknown) {
    StateStore store;
    const auto c = store.combined();
    EXPECT_FALSE(c.arm_present());
    EXPECT_FALSE(c.hand_present());
    EXPECT_EQ(c.sync_quality, SyncQuality::Unknown);
}

// 断连视同手断连：arm 失效后，残留的有效 hand 快照按缺席处理
TEST(StateStoreSeqTest, HandDependsOnArmDisconnect) {
    StateStore store;
    RobotArmState a;
    a.valid = true;
    DexterousHandState h;
    h.valid = true;
    store.update_arm(a);
    store.update_hand(h);
    EXPECT_EQ(store.combined().sync_quality, SyncQuality::Good);

    // RM75 断电 → arm 失效（O6 由 RM 末端供电，同步断电）
    RobotArmState gone;
    gone.valid = false;
    store.update_arm(gone);

    const auto c = store.combined();
    EXPECT_FALSE(c.arm_present());
    EXPECT_FALSE(c.hand_present());  // 残留 hand 快照不产生"arm 无、hand 有"
    EXPECT_EQ(c.sync_quality, SyncQuality::Unknown);
}
