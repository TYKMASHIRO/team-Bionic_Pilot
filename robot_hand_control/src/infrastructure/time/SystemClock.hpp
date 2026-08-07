#pragma once

#include "robotics/interfaces/IClock.hpp"

namespace robotics::infra {

/**
 * @brief 基于真实系统时钟的 IClock 实现。
 */
class SystemClock final : public domain::IClock {
public:
    std::chrono::steady_clock::time_point steady_now() override;
    std::chrono::system_clock::time_point system_now() override;
    domain::Timestamp now() override;
};

}  // namespace robotics::infra
