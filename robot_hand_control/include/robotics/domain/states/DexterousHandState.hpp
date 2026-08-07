#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "robotics/domain/types/DeviceType.hpp"
#include "robotics/domain/types/JointVector.hpp"
#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/**
 * @brief O6 灵巧手状态快照（6 通道）。
 * position 为 raw 0-255（小值弯曲、大值伸直），
 * velocity/torque 为 raw 0-255；temperature ℃；fault_code 0=无故障。
 */
struct DexterousHandState {
    std::array<double, kHandDof> position{};
    std::array<double, kHandDof> velocity{};
    std::array<double, kHandDof> torque{};
    std::array<double, kHandDof> temperature{};
    std::array<std::uint8_t, kHandDof> fault_code{};

    // 压力数据（5 指 × 行 × 列，O6=4×10）；无压力硬件时为空
    std::vector<std::vector<std::vector<uint8_t>>> pressure;

    HandState state = HandState::Unknown;
    bool valid = false;
    bool fresh = false;
    Timestamp timestamp;
    std::uint64_t sequence = 0;

    HandJointVector position_vector() const { return position; }
};

}  // namespace robotics::domain
