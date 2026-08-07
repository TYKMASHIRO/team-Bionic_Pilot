#include "src/services/StateStore.hpp"

namespace robotics::domain {

namespace {
constexpr std::int64_t kGoodSyncThresholdNs = 50'000'000;  // 50ms
}

void StateStore::update_arm(const RobotArmState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    arm_ = state;
}

void StateStore::update_hand(const DexterousHandState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    hand_ = state;
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
    if (arm_) {
        c.arm = *arm_;
        c.timestamp = arm_->timestamp;
    }
    if (hand_) {
        c.hand = *hand_;
        if (arm_) {
            c.time_delta_ns = hand_->timestamp.steady_ns() - arm_->timestamp.steady_ns();
            c.sync_quality =
                (std::abs(c.time_delta_ns) <= kGoodSyncThresholdNs)
                    ? SyncQuality::Good
                    : SyncQuality::Skewed;
        } else {
            c.sync_quality = SyncQuality::Unknown;
        }
        if (!arm_) c.timestamp = hand_->timestamp;
    }
    c.sequence = ++combined_seq_;
    return c;
}

}  // namespace robotics::domain
