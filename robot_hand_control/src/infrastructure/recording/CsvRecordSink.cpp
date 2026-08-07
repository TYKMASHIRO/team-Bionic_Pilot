#include "src/infrastructure/recording/CsvRecordSink.hpp"

#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

#include "src/domain/recording/CsvStateColumns.hpp"

namespace robotics::infra {

namespace {
namespace fs = std::filesystem;
}

// ---------------------------------------------------------------------------
// 构造/析构
// ---------------------------------------------------------------------------
CsvRecordSink::CsvRecordSink(const std::string& out_dir,
                             const domain::RecordingMetadata& meta)
    : dir_(out_dir + "/" + meta.session_id), meta_(meta) {
    good_ = open_files();
}

CsvRecordSink::~CsvRecordSink() {
    if (good_) flush();
}

bool CsvRecordSink::open_files() {
    std::error_code ec;
    fs::create_directories(dir_, ec);
    if (ec) return false;

    states_.open(dir_ + "/states.csv", std::ios::out | std::ios::trunc);
    events_.open(dir_ + "/events.csv", std::ios::out | std::ios::trunc);
    commands_.open(dir_ + "/commands.csv", std::ios::out | std::ios::trunc);
    metadata_.open(dir_ + "/metadata.txt", std::ios::out | std::ios::trunc);
    if (!states_ || !events_ || !commands_ || !metadata_) return false;

    // 表头与数据行同源（CsvStateColumns），保证列数一致。
    domain::CombinedRobotState dummy;
    if (!domain::CsvStateColumns::write_row(states_, dummy, /*header=*/true)) {
        return false;
    }
    states_ << '\n';
    if (!states_) return false;

    // metadata 静态块
    metadata_ << "session_id=" << meta_.session_id << '\n'
              << "format_version=" << meta_.format_version << '\n'
              << "config_hash=" << meta_.config_hash << '\n'
              << "sample_rate_hz=" << meta_.sample_rate_hz << '\n'
              << "calibration_ref=" << meta_.calibration_ref << '\n'
              << "absent_encoding=valid_false->numeric 0 / string empty\n"
              << "start_iso=" << meta_.start_time.to_iso8601() << '\n'
              << "start_steady_ns=" << meta_.start_time.steady_ns() << '\n';
    return true;
}

// ---------------------------------------------------------------------------
// IRecordSink
// ---------------------------------------------------------------------------
bool CsvRecordSink::write_state(const domain::CombinedRobotState& state) {
    if (!good_) return false;
    if (!domain::CsvStateColumns::write_row(states_, state, /*header=*/false)) {
        return false;
    }
    states_ << '\n';
    if (!states_) return false;
    {
        std::lock_guard<std::mutex> lock(counter_mutex_);
        ++rows_written_;
    }
    return true;
}

bool CsvRecordSink::write_event(domain::Timestamp ts, const std::string& event) {
    if (!good_) return false;
    events_ << ts.to_iso8601() << ',' << ts.wall_ns() << ',' << ts.steady_ns() << ','
            << domain::csv_escape_shared(event) << '\n';
    if (!events_) return false;
    {
        std::lock_guard<std::mutex> lock(counter_mutex_);
        ++events_written_;
    }
    return true;
}

bool CsvRecordSink::write_command(const std::string& command_json) {
    if (!good_) return false;
    const domain::Timestamp ts = domain::make_timestamp();
    commands_ << ts.to_iso8601() << ',' << ts.wall_ns() << ','
              << domain::csv_escape_shared(command_json) << '\n';
    if (!commands_) return false;
    {
        std::lock_guard<std::mutex> lock(counter_mutex_);
        ++commands_written_;
    }
    return true;
}

bool CsvRecordSink::flush() {
    if (!good_) return false;
    states_.flush();
    events_.flush();
    commands_.flush();

    std::lock_guard<std::mutex> lock(counter_mutex_);
    // 汇总块只写一次：Recorder::stop 与析构都会 flush，避免重复 end_iso/frames。
    if (!summary_written_) {
        summary_written_ = true;
        const domain::Timestamp now = domain::make_timestamp();
        metadata_ << "end_iso=" << now.to_iso8601() << '\n'
                  << "end_steady_ns=" << now.steady_ns() << '\n'
                  << "frames_recorded=" << rows_written_ << '\n'
                  << "events_recorded=" << events_written_ << '\n'
                  << "commands_recorded=" << commands_written_ << '\n'
                  << "dropped_frames=" << dropped_ << '\n';
        metadata_.flush();
    }
    return true;
}

void CsvRecordSink::add_dropped_frames(std::size_t count) {
    std::lock_guard<std::mutex> lock(counter_mutex_);
    dropped_ += count;
}

std::size_t CsvRecordSink::dropped_frames() const {
    std::lock_guard<std::mutex> lock(counter_mutex_);
    return dropped_;
}

std::uint64_t CsvRecordSink::rows_written() const {
    std::lock_guard<std::mutex> lock(counter_mutex_);
    return rows_written_;
}

}  // namespace robotics::infra
