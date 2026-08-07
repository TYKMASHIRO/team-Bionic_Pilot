#pragma once

#include <optional>

#include "robotics/domain/states/CombinedRobotState.hpp"

namespace robotics::domain {

/**
 * @brief 统一状态中心。
 * 采集线程写入最新快照；读取方获取不可变副本。
 * 实现需线程安全。
 */
class IStateStore {
public:
    virtual ~IStateStore() = default;

    /// 更新机械臂快照
    virtual void update_arm(const RobotArmState& state) = 0;
    /// 更新灵巧手快照
    virtual void update_hand(const DexterousHandState& state) = 0;

    /// 读取最新机械臂快照
    virtual std::optional<RobotArmState> latest_arm() const = 0;
    /// 读取最新灵巧手快照
    virtual std::optional<DexterousHandState> latest_hand() const = 0;

    /// 组装组合快照（含时间差与同步质量）
    virtual CombinedRobotState combined() const = 0;
};

}  // namespace robotics::domain
