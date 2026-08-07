#include "src/services/StateStore.hpp"

namespace robotics::domain {

namespace {
constexpr std::int64_t kGoodSyncThresholdNs = 50'000'000;  // 50ms
}

void StateStore::update_arm(const RobotArmState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_ = state;
    // 统一打序号：设备适配器一律 sequence=0，序号单一来源在本 StateStore。
    arm_->sequence = ++arm_seq_;
}

void StateStore::update_hand(const DexterousHandState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    hand_ = state;
    // 统一打序号（见 update_arm）。
    hand_->sequence = ++hand_seq_;
}

std::optional<RobotArmState> StateStore::latest_arm() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return arm_;
}

std::optional<DexterousHandState> StateStore::latest_hand() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return hand_;
}

CombinedRobotState StateStore::combined() const {
    std::lock_guard<std::mutex> lock(mutex_);
    CombinedRobotState c;

    // O6 由 RM75 末端供电：RM75 断电/故障即 O6 断电。
    // 因此 hand 在场必须以 arm 在场为前提——即使 hand_ 残留一次有效快照
    // （RM 断连前最后一次读取），combined() 也视同手断连，不产生"arm 无、hand 有"
    // 的物理不可能组合行。
    const bool arm_ok = arm_.has_value() && arm_->valid;
    const bool hand_ok = hand_.has_value() && hand_->valid && arm_ok;

    if (arm_ok) {
        c.arm = *arm_;
        c.timestamp = arm_->timestamp;
    }
    if (hand_ok) {
        c.hand = *hand_;
        c.time_delta_ns = hand_->timestamp.steady_ns() - arm_->timestamp.steady_ns();
        c.sync_quality =
            (std::abs(c.time_delta_ns) <= kGoodSyncThresholdNs)
                ? SyncQuality::Good
                : SyncQuality::Skewed;
    } else if (arm_ok) {
        // arm 在场、hand 缺位/失效 → 同步丢失
        c.sync_quality = SyncQuality::Lost;
    }
    // 两设备均不在场 → SyncQuality::Unknown（保持默认）

    c.sequence = ++combined_seq_;
    return c;
}

}  // namespace robotics::domain
