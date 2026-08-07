#include "src/domain/recording/CsvStateColumns.hpp"

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <sstream>

namespace robotics::domain {

std::string csv_escape_shared(const std::string& v) {
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

namespace {

/// 逐列拼接器（CSV 行）。
class RowBuilder {
public:
    RowBuilder() { oss_ << std::setprecision(10) << std::defaultfloat; }

    void i64(std::int64_t v) { field(std::to_string(v)); }
    void u64(std::uint64_t v) { field(std::to_string(v)); }
    void d(double v) { field(to_double(v)); }
    void s(const std::string& v) { field(csv_escape_shared(v)); }
    void raw(const std::string& v) { field(v); }

    std::string str() const { return oss_.str(); }

private:
    void field(const std::string& v) {
        if (!first_) oss_ << ',';
        first_ = false;
        oss_ << v;
    }
    /// 与旧实现一致：double 用默认流精度（10 位）输出，避免 -0.000000
    static std::string to_double(double v) {
        std::ostringstream s;
        s << std::setprecision(10) << std::defaultfloat << v;
        return s.str();
    }
    std::ostringstream oss_;
    bool first_ = true;  // 首列前不加逗号
};

/// 构建列名序列（顺序即表头/数据行列序）。
std::vector<std::string> build_names() {
    std::vector<std::string> n;
    auto add = [&n](const char* s) { n.emplace_back(s); };
    auto add_loop = [&n](const char* fmt, std::size_t cnt) {
        char buf[32];
        for (std::size_t i = 0; i < cnt; ++i) {
            std::snprintf(buf, sizeof(buf), fmt, i);
            n.emplace_back(buf);
        }
    };

    // ---- 全局（8）----
    add("seq");
    add("t_steady_ns");
    add("t_wall_ns");
    add("t_wall_iso");
    add("sync_quality");
    add("time_delta_ns");
    add("skill");
    add("cmd_id");

    // ---- arm（56）----
    add("arm_valid");
    add("arm_fresh");
    add("arm_seq");
    add("arm_t_steady_ns");
    add("arm_t_wall_ns");
    add_loop("arm_j_pos_%zu", kArmDof);
    add_loop("arm_j_vel_%zu", kArmDof);
    add_loop("arm_j_cur_%zu", kArmDof);
    add_loop("arm_j_temp_%zu", kArmDof);
    add_loop("arm_j_en_%zu", kArmDof);
    add_loop("arm_j_err_%zu", kArmDof);
    add("arm_j_valid");
    add("arm_tcp_x");
    add("arm_tcp_y");
    add("arm_tcp_z");
    add("arm_tcp_rx");
    add("arm_tcp_ry");
    add("arm_tcp_rz");
    add("arm_fx");
    add("arm_fy");
    add("arm_fz");
    add("arm_mx");
    add("arm_my");
    add("arm_mz");
    add("arm_motion_state");
    add("arm_reached");
    add("arm_sys_err");

    // ---- hand（46）----
    add("hand_valid");
    add("hand_fresh");
    add("hand_seq");
    add("hand_t_steady_ns");
    add("hand_t_wall_ns");
    add_loop("hand_pos_%zu", kHandDof);
    add_loop("hand_vel_%zu", kHandDof);
    add_loop("hand_torq_%zu", kHandDof);
    add_loop("hand_temp_%zu", kHandDof);
    add_loop("hand_fault_%zu", kHandDof);
    add("hand_state");
    add("hand_pressure_shape");
    add("hand_pressure_n");
    add("hand_pressure");

    return n;
}

}  // namespace

const std::vector<std::string>& CsvStateColumns::names() {
    static const std::vector<std::string> kNames = build_names();
    return kNames;
}

bool CsvStateColumns::write_row(std::ostream& os, const CombinedRobotState& c,
                                bool header) {
    const auto& nm = names();
    RowBuilder b;
    if (header) {
        for (const auto& col : nm) b.raw(col);
        os << b.str();
        return static_cast<bool>(os);
    }

    auto num_col = [&b](double v) { b.d(v); };
    auto int_col = [&b](std::int64_t v) { b.i64(v); };
    auto u64_col = [&b](std::uint64_t v) { b.u64(v); };
    auto str_col = [&b](const std::string& v) { b.s(v); };

    // ---- 全局 ----
    u64_col(c.sequence);
    int_col(c.timestamp.steady_ns());
    int_col(c.timestamp.wall_ns());
    str_col(c.timestamp.to_iso8601());
    int_col(static_cast<std::int64_t>(c.sync_quality));
    int_col(c.time_delta_ns);
    str_col(c.current_skill);
    str_col(c.current_command_id);

    // ---- arm ----
    const bool arm_ok = c.arm.valid;
    num_col(arm_ok ? 1 : 0);
    num_col(c.arm.fresh ? 1 : 0);
    u64_col(c.arm.sequence);
    int_col(c.arm.timestamp.steady_ns());
    int_col(c.arm.timestamp.wall_ns());

    const auto& js = c.arm.joint_state;
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok ? js.position[i] : 0.0);
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok ? js.velocity[i] : 0.0);
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok ? js.current[i] : 0.0);
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok ? js.temperature[i] : 0.0);
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok && js.enabled[i] ? 1 : 0);
    for (std::size_t i = 0; i < domain::kArmDof; ++i)
        num_col(arm_ok ? static_cast<double>(js.error_code[i]) : 0.0);
    num_col(arm_ok && js.valid ? 1 : 0);

    num_col(arm_ok ? c.arm.tcp_pose.position.x : 0.0);
    num_col(arm_ok ? c.arm.tcp_pose.position.y : 0.0);
    num_col(arm_ok ? c.arm.tcp_pose.position.z : 0.0);
    num_col(arm_ok ? c.arm.tcp_pose.orientation.rx : 0.0);
    num_col(arm_ok ? c.arm.tcp_pose.orientation.ry : 0.0);
    num_col(arm_ok ? c.arm.tcp_pose.orientation.rz : 0.0);

    num_col(arm_ok ? c.arm.force_torque.force[0] : 0.0);
    num_col(arm_ok ? c.arm.force_torque.force[1] : 0.0);
    num_col(arm_ok ? c.arm.force_torque.force[2] : 0.0);
    num_col(arm_ok ? c.arm.force_torque.torque[0] : 0.0);
    num_col(arm_ok ? c.arm.force_torque.torque[1] : 0.0);
    num_col(arm_ok ? c.arm.force_torque.torque[2] : 0.0);

    int_col(static_cast<std::int64_t>(c.arm.motion_state));
    num_col(c.arm.reached_target ? 1 : 0);

    std::string sys_err;
    if (arm_ok && !c.arm.system_errors.empty()) {
        for (std::size_t i = 0; i < c.arm.system_errors.size(); ++i) {
            if (i) sys_err += ';';
            sys_err += std::to_string(c.arm.system_errors[i]);
        }
    }
    str_col(sys_err);

    // ---- hand ----
    const bool hand_ok = c.hand.valid;
    num_col(hand_ok ? 1 : 0);
    num_col(c.hand.fresh ? 1 : 0);
    u64_col(c.hand.sequence);
    int_col(c.hand.timestamp.steady_ns());
    int_col(c.hand.timestamp.wall_ns());

    for (std::size_t i = 0; i < domain::kHandDof; ++i)
        num_col(hand_ok ? c.hand.position[i] : 0.0);
    for (std::size_t i = 0; i < domain::kHandDof; ++i)
        num_col(hand_ok ? c.hand.velocity[i] : 0.0);
    for (std::size_t i = 0; i < domain::kHandDof; ++i)
        num_col(hand_ok ? c.hand.torque[i] : 0.0);
    for (std::size_t i = 0; i < domain::kHandDof; ++i)
        num_col(hand_ok ? c.hand.temperature[i] : 0.0);
    for (std::size_t i = 0; i < domain::kHandDof; ++i)
        num_col(hand_ok ? static_cast<double>(c.hand.fault_code[i]) : 0.0);

    int_col(static_cast<std::int64_t>(c.hand.state));

    // 压力：pressure[finger][row][col]（3 层）。
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
    str_col(shape);
    u64_col(pressure_n);
    str_col(flat);

    os << b.str();
    return static_cast<bool>(os);
}

std::vector<std::string> csv_split(const std::string& line) {
    std::vector<std::string> out;
    std::size_t start = 0;
    while (true) {
        const std::size_t comma = line.find(',', start);
        if (comma == std::string::npos) {
            out.push_back(line.substr(start));
            break;
        }
        out.push_back(line.substr(start, comma - start));
        start = comma + 1;
    }
    return out;
}

namespace {

/// 表头列名 → 列索引（找不到返回 npos）。
using HeaderIndex = std::vector<std::pair<std::string, std::size_t>>;

std::size_t find_col(const HeaderIndex& idx, const char* name) {
    for (const auto& [n, i] : idx) {
        if (n == name) return i;
    }
    return std::string::npos;
}

/// 数值解析：返回 false 表示字段为空或非法（非有限值计入，但空串按 0）。
bool parse_double(const std::string& s, double& out) {
    if (s.empty()) {
        out = 0.0;
        return true;
    }
    errno = 0;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') return false;  // 非数字
    out = v;
    return true;
}

bool parse_uint64(const std::string& s, std::uint64_t& out) {
    if (s.empty()) {
        out = 0;
        return true;
    }
    errno = 0;
    char* end = nullptr;
    const unsigned long long v = std::strtoull(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return false;
    out = static_cast<std::uint64_t>(v);
    return true;
}

bool parse_int64(const std::string& s, std::int64_t& out) {
    if (s.empty()) {
        out = 0;
        return true;
    }
    errno = 0;
    char* end = nullptr;
    const long long v = std::strtoll(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return false;
    out = static_cast<std::int64_t>(v);
    return true;
}

}  // namespace

bool CsvStateColumns::parse_row(const std::vector<std::string>& header,
                                const std::string& line,
                                CombinedRobotState& out) {
    HeaderIndex idx;
    idx.reserve(header.size());
    for (std::size_t i = 0; i < header.size(); ++i) idx.emplace_back(header[i], i);

    const auto cols = csv_split(line);
    if (cols.size() < header.size()) return false;  // 字段数不足

    // 工具 lambda：按列名取字符串 / double / uint64 / int64
    auto col_str = [&](const char* name) -> const std::string& {
        const std::size_t i = find_col(idx, name);
        static const std::string kEmpty;
        return i == std::string::npos ? kEmpty : cols[i];
    };
    auto col_dbl = [&](const char* name, double& v) {
        const std::size_t i = find_col(idx, name);
        double x = 0.0;
        if (i == std::string::npos) return false;
        if (!parse_double(cols[i], x)) return false;
        v = x;
        return true;
    };
    auto col_u64 = [&](const char* name, std::uint64_t& v) {
        const std::size_t i = find_col(idx, name);
        if (i == std::string::npos) return false;
        return parse_uint64(cols[i], v);
    };
    auto col_i64 = [&](const char* name, std::int64_t& v) {
        const std::size_t i = find_col(idx, name);
        if (i == std::string::npos) return false;
        return parse_int64(cols[i], v);
    };

    // ---- 全局 ----
    std::uint64_t seq = 0;
    std::int64_t t_steady_ns = 0, t_wall_ns = 0;
    if (!col_u64("seq", seq)) return false;
    if (!col_i64("t_steady_ns", t_steady_ns)) return false;
    if (!col_i64("t_wall_ns", t_wall_ns)) return false;
    std::int64_t sync_q = 0;
    col_i64("sync_quality", sync_q);
    col_i64("time_delta_ns", out.time_delta_ns);
    out.current_skill = col_str("skill");
    out.current_command_id = col_str("cmd_id");
    out.sequence = seq;
    out.timestamp.steady = std::chrono::steady_clock::time_point{
        std::chrono::nanoseconds(t_steady_ns)};
    out.timestamp.wall = std::chrono::system_clock::time_point{
        std::chrono::nanoseconds(t_wall_ns)};
    out.sync_quality = static_cast<SyncQuality>(sync_q);

    // ---- arm ----
    double arm_valid = 0;
    if (!col_dbl("arm_valid", arm_valid)) return false;
    const bool arm_ok = arm_valid != 0.0;
    out.arm.valid = arm_ok;
    col_dbl("arm_fresh", arm_valid);  // 复用临时
    out.arm.fresh = arm_valid != 0.0;
    std::uint64_t arm_seq = 0;
    col_u64("arm_seq", arm_seq);
    out.arm.sequence = arm_seq;
    std::int64_t arm_ts_ns = 0;
    col_i64("arm_t_steady_ns", arm_ts_ns);
    out.arm.timestamp.steady = std::chrono::steady_clock::time_point{
        std::chrono::nanoseconds(arm_ts_ns)};
    col_i64("arm_t_wall_ns", arm_ts_ns);
    out.arm.timestamp.wall = std::chrono::system_clock::time_point{
        std::chrono::nanoseconds(arm_ts_ns)};

    auto& js = out.arm.joint_state;
    char buf[32];
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_pos_%zu", i);
        if (!col_dbl(buf, js.position[i])) return false;
        if (!arm_ok) js.position[i] = 0.0;
    }
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_vel_%zu", i);
        if (!col_dbl(buf, js.velocity[i])) return false;
        if (!arm_ok) js.velocity[i] = 0.0;
    }
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_cur_%zu", i);
        if (!col_dbl(buf, js.current[i])) return false;
        if (!arm_ok) js.current[i] = 0.0;
    }
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_temp_%zu", i);
        if (!col_dbl(buf, js.temperature[i])) return false;
        if (!arm_ok) js.temperature[i] = 0.0;
    }
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_en_%zu", i);
        double en = 0;
        if (!col_dbl(buf, en)) return false;
        js.enabled[i] = en != 0.0 && arm_ok;
    }
    for (std::size_t i = 0; i < kArmDof; ++i) {
        std::snprintf(buf, sizeof(buf), "arm_j_err_%zu", i);
        double err = 0;
        if (!col_dbl(buf, err)) return false;
        js.error_code[i] = static_cast<std::uint16_t>(err);
    }
    double arm_j_valid = 0;
    col_dbl("arm_j_valid", arm_j_valid);
    js.valid = arm_ok && arm_j_valid != 0.0;

    double x = 0;
    if (!col_dbl("arm_tcp_x", x)) return false;
    out.arm.tcp_pose.position.x = arm_ok ? x : 0.0;
    if (!col_dbl("arm_tcp_y", x)) return false;
    out.arm.tcp_pose.position.y = arm_ok ? x : 0.0;
    if (!col_dbl("arm_tcp_z", x)) return false;
    out.arm.tcp_pose.position.z = arm_ok ? x : 0.0;
    if (!col_dbl("arm_tcp_rx", x)) return false;
    out.arm.tcp_pose.orientation.rx = arm_ok ? x : 0.0;
    if (!col_dbl("arm_tcp_ry", x)) return false;
    out.arm.tcp_pose.orientation.ry = arm_ok ? x : 0.0;
    if (!col_dbl("arm_tcp_rz", x)) return false;
    out.arm.tcp_pose.orientation.rz = arm_ok ? x : 0.0;

    if (!col_dbl("arm_fx", x)) return false;
    out.arm.force_torque.force[0] = arm_ok ? x : 0.0;
    if (!col_dbl("arm_fy", x)) return false;
    out.arm.force_torque.force[1] = arm_ok ? x : 0.0;
    if (!col_dbl("arm_fz", x)) return false;
    out.arm.force_torque.force[2] = arm_ok ? x : 0.0;
    if (!col_dbl("arm_mx", x)) return false;
    out.arm.force_torque.torque[0] = arm_ok ? x : 0.0;
    if (!col_dbl("arm_my", x)) return false;
    out.arm.force_torque.torque[1] = arm_ok ? x : 0.0;
    if (!col_dbl("arm_mz", x)) return false;
    out.arm.force_torque.torque[2] = arm_ok ? x : 0.0;

    std::int64_t motion = 0;
    if (!col_i64("arm_motion_state", motion)) return false;
    out.arm.motion_state = static_cast<ArmMotionState>(motion);
    col_dbl("arm_reached", x);
    out.arm.reached_target = arm_ok && x != 0.0;

    const std::string& sys_err = col_str("arm_sys_err");
    out.arm.system_errors.clear();
    if (arm_ok && !sys_err.empty()) {
        std::size_t start = 0;
        while (start <= sys_err.size()) {
            const std::size_t semi = sys_err.find(';', start);
            const std::string tok =
                sys_err.substr(start, semi == std::string::npos ? std::string::npos
                                                                : semi - start);
            if (!tok.empty()) out.arm.system_errors.push_back(std::atoi(tok.c_str()));
            if (semi == std::string::npos) break;
            start = semi + 1;
        }
    }

    // ---- hand ----
    double hand_valid = 0;
    if (!col_dbl("hand_valid", hand_valid)) return false;
    const bool hand_ok = hand_valid != 0.0;
    out.hand.valid = hand_ok;
    col_dbl("hand_fresh", x);
    out.hand.fresh = hand_ok && x != 0.0;
    std::uint64_t hand_seq = 0;
    col_u64("hand_seq", hand_seq);
    out.hand.sequence = hand_seq;
    std::int64_t hand_ts = 0;
    col_i64("hand_t_steady_ns", hand_ts);
    out.hand.timestamp.steady = std::chrono::steady_clock::time_point{
        std::chrono::nanoseconds(hand_ts)};
    col_i64("hand_t_wall_ns", hand_ts);
    out.hand.timestamp.wall = std::chrono::system_clock::time_point{
        std::chrono::nanoseconds(hand_ts)};

    for (std::size_t i = 0; i < kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_pos_%zu", i);
        if (!col_dbl(buf, out.hand.position[i])) return false;
        if (!hand_ok) out.hand.position[i] = 0.0;
    }
    for (std::size_t i = 0; i < kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_vel_%zu", i);
        if (!col_dbl(buf, out.hand.velocity[i])) return false;
        if (!hand_ok) out.hand.velocity[i] = 0.0;
    }
    for (std::size_t i = 0; i < kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_torq_%zu", i);
        if (!col_dbl(buf, out.hand.torque[i])) return false;
        if (!hand_ok) out.hand.torque[i] = 0.0;
    }
    for (std::size_t i = 0; i < kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_temp_%zu", i);
        if (!col_dbl(buf, out.hand.temperature[i])) return false;
        if (!hand_ok) out.hand.temperature[i] = 0.0;
    }
    for (std::size_t i = 0; i < kHandDof; ++i) {
        std::snprintf(buf, sizeof(buf), "hand_fault_%zu", i);
        double f = 0;
        if (!col_dbl(buf, f)) return false;
        out.hand.fault_code[i] = hand_ok ? static_cast<std::uint8_t>(f) : 0;
    }

    std::int64_t hand_state = 0;
    col_i64("hand_state", hand_state);
    out.hand.state = static_cast<HandState>(hand_state);

    // 压力（absent 留空）
    const std::string& shape = col_str("hand_pressure_shape");
    const std::string& flat = col_str("hand_pressure");
    std::uint64_t p_n = 0;
    col_u64("hand_pressure_n", p_n);
    out.hand.pressure.clear();
    if (hand_ok && !shape.empty() && !flat.empty()) {
        const auto dims = csv_split(shape);
        std::vector<std::size_t> dim(3, 0);
        for (std::size_t i = 0; i < dims.size() && i < 3; ++i)
            dim[i] = static_cast<std::size_t>(std::atoll(dims[i].c_str()));
        std::vector<std::string> vals;
        std::size_t start = 0;
        while (start <= flat.size()) {
            const std::size_t semi = flat.find(';', start);
            vals.push_back(flat.substr(start, semi == std::string::npos
                                                 ? std::string::npos
                                                 : semi - start));
            if (semi == std::string::npos) break;
            start = semi + 1;
        }
        std::size_t k = 0;
        for (std::size_t f = 0; f < dim[0]; ++f) {
            std::vector<std::vector<std::uint8_t>> finger;
            for (std::size_t r = 0; r < dim[1]; ++r) {
                std::vector<std::uint8_t> row;
                for (std::size_t cc = 0; cc < dim[2]; ++cc) {
                    const std::uint8_t v =
                        (k < vals.size()) ? static_cast<std::uint8_t>(
                                                std::atoi(vals[k].c_str()))
                                          : 0;
                    row.push_back(v);
                    ++k;
                }
                finger.push_back(std::move(row));
            }
            out.hand.pressure.push_back(std::move(finger));
        }
    }

    return true;
}

}  // namespace robotics::domain
