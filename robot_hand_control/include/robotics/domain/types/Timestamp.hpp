#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace robotics::domain {

/**
 * @brief 统一时间戳。
 *
 * 控制时序使用 steady_clock（单调，不受系统时间调整影响）；
 * 日志/文件名使用 system_clock。两者不得混用。
 */
struct Timestamp {
    // 单调时钟：控制时序、新鲜度判断
    std::chrono::steady_clock::time_point steady{};

    // 系统时钟：日志与文件命名
    std::chrono::system_clock::time_point wall{};

    // 自上次复位以来的单调纳秒计数（跨设备同步用）
    std::int64_t steady_ns() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   steady.time_since_epoch())
            .count();
    }

    // 墙钟纳秒（自 1970-01-01）
    std::int64_t wall_ns() const {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   wall.time_since_epoch())
            .count();
    }

    /** 墙钟格式化为 ISO8601 字符串（本地时间）。 */
    std::string to_iso8601() const;
};

/** 当前时刻的统一时间戳。 */
Timestamp make_timestamp();

}  // namespace robotics::domain
