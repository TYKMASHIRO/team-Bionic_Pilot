#include "src/infrastructure/recording/CsvRecordSink.hpp"

#include <cstdio>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <vector>

namespace robotics::infra {

namespace {
namespace fs = std::filesystem;

/// CSV 字段转义：含逗号/引号/换行时用双引号包裹，内部引号加倍。
std::string csv_escape(const std::string& v) {
    if (v.find(',') == std::string::npos &&
        v.find('"') == std::string::npos &&
        v.find('\n') == std::string::npos) {
        return v;
    }
    std::string out;
    out.reserve(v.size() + 2);
    out.push_back('"');
    for (char ch : v) {
        if (ch == '"') out.push_back('"');
        out.push_back(ch);
    }
    out.push_back('"');
    return out;
}

/// 逐列拼接器（CSV 行）。
class RowBuilder {
public:
    RowBuilder() { oss_ << std::setprecision(10) << std::defaultfloat; }

    void i64(std::int64_t v) { field(std::to_string(v)); }
    void u64(std::uint64_t v) { field(std::to_string(v)); }
    void d(double v) {
        if (!first_) oss_ << ',';
        first_ = false;
        oss_ << v;
    }
    void s(const std::string& v) { field(csv_escape(v)); }
    void raw(const std::string& v) { field(v); }

    std::string str() const { return oss_.str(); }

private:
    void field(const std::string& v) {
        if (!first_) oss_ << ',';
        first_ = false;
        oss_ << v;
    }
    std::ostringstream oss_;
    bool first_ = true;  // 首列前不加逗号
};

/// 表头与数据行共用同一列序列（header=true 输出列名，否则输出值）。
/// 保证 states.csv 列数恒定、表头与行永远对齐。
void build_state_row(RowBuilder& b, const domain::CombinedRobotState& c,
                     bool header) {
    auto num_col = [&](const char* name, double v) {
        if (header) b.raw(name); else b.d(v);
    };
    auto int_col = [&](const char* name, std::int64_t v) {
        if (header) b.raw(name); else b.i64(v);
    };
    auto u64_col = [&](const char* name, std::uint64_t v) {
        if (header) b.raw(name); else b.u64(v);
    };
    auto str_col = [&](const char* name, const std::string& v) {
        if (header) b.raw(name); else b.s(v);
    };

    // ---- 全局 ----
    u64_col("seq", c.sequence);
    int_col("t_steady_ns", c.timestamp.steady_ns());
    int_col("t_wall_ns", c.timestamp.wall_ns());
    str_col("t_wall_iso", c.timestamp.to_iso8601());
    int_col("sync_quality", static_cast<std::int64_t>(c.sync_quality));
    int_col("time_delta_ns", c.time_delta_ns);
    str_col("skill", c.current_skill);
    str_col("cmd_id", c.current_command_id);

    // ---- arm ----
    const bool arm_ok = c.arm.valid;
    num_col("arm_valid", arm_ok ? 1 : 0);
    num_col("arm_fresh", c.arm.fresh ? 1 : 0);
    u64_col("arm_seq", c.arm.sequence);
    int_col("arm_t_steady_ns", c.arm.timestamp.steady_ns());
    int_col("arm_t_wall_ns", c.arm.timestamp.wall_ns());

    const auto& js = c.arm.joint_state;
    char buf[32];
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_pos_%zu", i);
        num_col(buf, arm_ok ? js.position[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_vel_%zu", i);
        num_col(buf, arm_ok ? js.velocity[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_cur_%zu", i);
        num_col(buf, arm_ok ? js.current[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_temp_%zu", i);
        num_col(buf, arm_ok ? js.temperature[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_en_%zu", i);
        num_col(buf, arm_ok && js.enabled[i] ? 1 : 0);
    }
    for (std::size_t i = 0; i < domain::kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_err_%zu", i);
        num_col(buf, arm_ok ? static_cast<double>(js.error_code[i]) : 0.0);
    }
    num_col("arm_j_valid", arm_ok && js.valid ? 1 : 0);

    num_col("arm_tcp_x", arm_ok ? c.arm.tcp_pose.position.x : 0.0);
    num_col("arm_tcp_y", arm_ok ? c.arm.tcp_pose.position.y : 0.0);
    num_col("arm_tcp_z", arm_ok ? c.arm.tcp_pose.position.z : 0.0);
    num_col("arm_tcp_rx", arm_ok ? c.arm.tcp_pose.orientation.rx : 0.0);
    num_col("arm_tcp_ry", arm_ok ? c.arm.tcp_pose.orientation.ry : 0.0);
    num_col("arm_tcp_rz", arm_ok ? c.arm.tcp_pose.orientation.rz : 0.0);

    num_col("arm_fx", arm_ok ? c.arm.force_torque.force[0] : 0.0);
    num_col("arm_fy", arm_ok ? c.arm.force_torque.force[1] : 0.0);
    num_col("arm_fz", arm_ok ? c.arm.force_torque.force[2] : 0.0);
    num_col("arm_mx", arm_ok ? c.arm.force_torque.torque[0] : 0.0);
    num_col("arm_my", arm_ok ? c.arm.force_torque.torque[1] : 0.0);
    num_col("arm_mz", arm_ok ? c.arm.force_torque.torque[2] : 0.0);

    int_col("arm_motion_state", static_cast<std::int64_t>(c.arm.motion_state));
    num_col("arm_reached", c.arm.reached_target ? 1 : 0);

    std::string sys_err;
    if (arm_ok && !c.arm.system_errors.empty()) {
        for (std::size_t i = 0; i < c.arm.system_errors.size(); ++i) {
            if (i) sys_err += ';';
            sys_err += std::to_string(c.arm.system_errors[i]);
        }
    }
    str_col("arm_sys_err", sys_err);

    // ---- hand ----
    const bool hand_ok = c.hand.valid;
    num_col("hand_valid", hand_ok ? 1 : 0);
    num_col("hand_fresh", c.hand.fresh ? 1 : 0);
    u64_col("hand_seq", c.hand.sequence);
    int_col("hand_t_steady_ns", c.hand.timestamp.steady_ns());
    int_col("hand_t_wall_ns", c.hand.timestamp.wall_ns());

    for (std::size_t i = 0; i < domain::kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_pos_%zu", i);
        num_col(buf, hand_ok ? c.hand.position[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_vel_%zu", i);
        num_col(buf, hand_ok ? c.hand.velocity[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_torq_%zu", i);
        num_col(buf, hand_ok ? c.hand.torque[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_temp_%zu", i);
        num_col(buf, hand_ok ? c.hand.temperature[i] : 0.0);
    }
    for (std::size_t i = 0; i < domain::kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_fault_%zu", i);
        num_col(buf, hand_ok ? static_cast<double>(c.hand.fault_code[i]) : 0.0);
    }

    int_col("hand_state", static_cast<std::int64_t>(c.hand.state));

    // 压力：pressure[finger][row][col]（3 层）。shape=指数,行数,列数；
    // 扁平化为"单元总数"个分号分隔值。
    std::string shape;
    std::uint64_t pressure_n = 0;
    std::string flat;
    if (hand_ok && !c.hand.pressure.empty()) {
        shape += std::to_string(c.hand.pressure.size());  // 指数量
        if (!c.hand.pressure[0].empty()) {
            shape += ",";
            shape += std::to_string(c.hand.pressure[0].size());  // 行数
            if (!c.hand.pressure[0][0].empty()) {
                shape += ",";
                shape += std::to_string(c.hand.pressure[0][0].size());  // 列数
            }
        }
        for (const auto& finger : c.hand.pressure) {
            for (const auto& row : finger) {
                for (const auto v : row) {
                    if (pressure_n) flat += ';';
                    ++pressure_n;
                    flat += std::to_string(static_cast<unsigned>(v));
                }
            }
        }
    }
    str_col("hand_pressure_shape", shape);
    u64_col("hand_pressure_n", pressure_n);
    str_col("hand_pressure", flat);
}

}  // namespace

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

    // 表头与行同源（build_state_row），保证列数一致
    domain::CombinedRobotState dummy;
    RowBuilder h;
    build_state_row(h, dummy, /*header=*/true);
    states_ << h.str() << '\n';
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
    RowBuilder b;
    build_state_row(b, state, /*header=*/false);
    states_ << b.str() << '\n';
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
            << csv_escape(event) << '\n';
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
              << csv_escape(command_json) << '\n';
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
