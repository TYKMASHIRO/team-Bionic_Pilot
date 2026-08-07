#pragma once

#include <mutex>
#include <optional>

#include "robotics/interfaces/IStateStore.hpp"

namespace robotics::domain {

/**
 * @brief 统一状态中心实现。
 * 采集线程写入最新快照（不可变副本），读取方获取快照。
 */
class StateStore : public IStateStore {
public:
    void update_arm(const RobotArmState& state) override;
    void update_hand(const DexterousHandState& state) override;

    std::optional<RobotArmState> latest_arm() const override;
    std::optional<DexterousHandState> latest_hand() const override;

    CombinedRobotState combined() const override;

private:
    mutable std::mutex mutex_;
    std::optional<RobotArmState> arm_;
    std::optional<DexterousHandState> hand_;
    mutable std::uint64_t combined_seq_ = 0;
};

}  // namespace robotics::domain
