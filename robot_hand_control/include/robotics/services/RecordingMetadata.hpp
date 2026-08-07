#pragma once

#include <atomic>
#include <cstdio>
#include <cstdint>
#include <ctime>
#include <string>

#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/// 当前录制文件格式版本
inline constexpr int kRecordingFormatVersion = 1;

/**
 * @brief 一次录制的元数据。
 * 写盘线程 start/flush 时使用；CsvRecordSink 落盘与 CLI 汇总均引用。
 */
struct RecordingMetadata {
    std::string session_id;
    int format_version = kRecordingFormatVersion;
    std::string config_hash;       ///< 配置 hash（来自 Configuration::config_hash）
    double sample_rate_hz = 0.0;   ///< 录制采样率（记录侧）
    std::string calibration_ref;   ///< 标定引用（阶段4 仅记录，不校验内容）

    Timestamp start_time;
    Timestamp end_time;

    std::uint64_t frames_recorded = 0;
    std::uint64_t events_recorded = 0;
    std::uint64_t commands_recorded = 0;
    std::uint64_t dropped_frames = 0;
};

/// 生成会话 ID：`rec_YYYYMMDD_HHMMSS_<原子序号>`（文件名安全，无冒号）。
inline std::string make_session_id(const Timestamp& ts) {
    static std::atomic<std::uint64_t> seq{0};
    std::time_t t = std::chrono::system_clock::to_time_t(ts.wall);
    std::tm tm{};
    localtime_r(&t, &tm);
    char buf[64];
    std::snprintf(buf, sizeof(buf), "rec_%04d%02d%02d_%02d%02d%02d_%llu",
                  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour,
                  tm.tm_min, tm.tm_sec,
                  static_cast<unsigned long long>(seq.fetch_add(1)));
    return buf;
}

}  // namespace robotics::domain
