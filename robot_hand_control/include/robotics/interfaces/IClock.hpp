#pragma once

#include <chrono>

#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/**
 * @brief 统一时钟抽象。
 * 封装单调时钟与系统时钟；测试可注入模拟时钟。
 */
class IClock {
public:
    virtual ~IClock() = default;

    /// 当前单调时间点（控制时序）
    virtual std::chrono::steady_clock::time_point steady_now() = 0;

    /// 当前系统时间点（日志/文件名）
    virtual std::chrono::system_clock::time_point system_now() = 0;

    /// 当前统一时间戳
    virtual Timestamp now() = 0;
};

}  // namespace robotics::domain
