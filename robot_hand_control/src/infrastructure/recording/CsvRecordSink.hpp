#pragma once

#include <cstdint>
#include <fstream>
#include <mutex>
#include <string>

#include "robotics/interfaces/IRecordSink.hpp"
#include "robotics/services/RecordingMetadata.hpp"

namespace robotics::infra {

/**
 * @brief CSV 落盘实现（domain::IRecordSink）。
 *
 * 一次录制输出到 <out_dir>/<session_id>/ 下 4 个文件：
 *   states.csv   每行一条 CombinedRobotState（固定 110 列，见 write_state）
 *   events.csv   t_wall_iso,t_wall_ns,t_steady_ns,event
 *   commands.csv t_wall_iso,t_wall_ns,command_json
 *   metadata.txt key=value（静态块 + flush 时追加汇总块）
 *
 * 设备缺席约定：valid=false 时该设备数值列写 0、字符串/矢量列写空串、*_valid=0；
 * 消费方以 *valid 列判定在场。
 *
 * 线程模型：ofstream 只由写盘线程（write_state/write_event/write_command/flush）
 * 访问，无需加锁；add_dropped_frames 可能来自采样线程，计数器独立加锁。
 */
class CsvRecordSink : public domain::IRecordSink {
public:
    CsvRecordSink(const std::string& out_dir, const domain::RecordingMetadata& meta);
    ~CsvRecordSink() override;

    CsvRecordSink(const CsvRecordSink&) = delete;
    CsvRecordSink& operator=(const CsvRecordSink&) = delete;

    /// 目录/文件是否成功打开（构造失败时 false，上层须检查后再启用录制）
    bool is_open() const { return good_; }

    // IRecordSink
    bool write_state(const domain::CombinedRobotState& state) override;
    bool write_event(domain::Timestamp ts, const std::string& event) override;
    bool write_command(const std::string& command_json) override;
    bool flush() override;
    void add_dropped_frames(std::size_t count) override;
    std::size_t dropped_frames() const override;

    /// 已写状态行数（汇总用）
    std::uint64_t rows_written() const;

private:
    bool open_files();

    std::string dir_;  ///< <out_dir>/<session_id>
    domain::RecordingMetadata meta_;
    bool good_ = false;

    std::ofstream states_;
    std::ofstream events_;
    std::ofstream commands_;
    std::ofstream metadata_;

    mutable std::mutex counter_mutex_;
    std::uint64_t rows_written_ = 0;
    std::uint64_t events_written_ = 0;
    std::uint64_t commands_written_ = 0;
    std::size_t dropped_ = 0;
    bool summary_written_ = false;  ///< metadata 汇总块只写一次（幂等 flush）
};

}  // namespace robotics::infra
