#include "src/trajectory/TrajectoryCsvLoader.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>

#include "robotics/services/RecordingMetadata.hpp"
#include "src/domain/recording/CsvStateColumns.hpp"

namespace robotics::domain {

namespace {
namespace fs = std::filesystem;

Result fail_load(const std::string& what) {
    return Result::fail(Error::make(
        ErrorCategory::Validation, DeviceType::Combined, "TrajectoryCsvLoader",
        1, "轨迹加载失败: " + what));
}

/// 读取文本文件全文；失败返回空串。
std::string read_file(const std::string& path) {
    std::ifstream f(path);
    if (!f) return {};
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// 解析 metadata.txt 的 key=value。
std::string meta_value(const std::string& text, const std::string& key) {
    const std::string needle = key + "=";
    const std::size_t pos = text.find(needle);
    if (pos == std::string::npos) return {};
    const std::size_t val = pos + needle.size();
    const std::size_t end = text.find('\n', val);
    return text.substr(val, end == std::string::npos ? std::string::npos
                                                     : end - val);
}

/// events.csv 行结构（iso,wall_ns,steady_ns,event）。
struct CsvEvent {
    std::int64_t steady_ns = 0;
    std::string text;
};

bool parse_events(const std::string& path, std::vector<CsvEvent>& out) {
    std::ifstream f(path);
    if (!f) return false;
    std::string line;
    while (std::getline(f, line)) {
        if (line.empty()) continue;
        const auto cols = csv_split(line);
        if (cols.size() < 4) continue;
        CsvEvent e;
        char* end = nullptr;
        e.steady_ns = std::strtoll(cols[2].c_str(), &end, 10);
        if (end == cols[2].c_str()) continue;
        e.text = cols[3];
        out.push_back(std::move(e));
    }
    // 按稳态时间排序，保证关联顺序
    std::sort(out.begin(), out.end(),
              [](const CsvEvent& a, const CsvEvent& b) {
                  return a.steady_ns < b.steady_ns;
              });
    return true;
}

}  // namespace

Result TrajectoryCsvLoader::load_recording(const std::string& recording_dir,
                                           Trajectory& out) {
    const fs::path dir(recording_dir);
    const fs::path states_path = dir / "states.csv";
    const fs::path events_path = dir / "events.csv";
    const fs::path meta_path = dir / "metadata.txt";

    std::ifstream states(states_path);
    if (!states) {
        return Result::fail(Error::make(
            ErrorCategory::Configuration, DeviceType::Combined,
            "TrajectoryCsvLoader", 2, "states.csv 不存在: " + states_path.string()));
    }

    // 可选：校验 metadata format_version
    const std::string md = read_file(meta_path.string());
    if (!md.empty()) {
        const std::string ver = meta_value(md, "format_version");
        if (!ver.empty()) {
            const int v = std::atoi(ver.c_str());
            if (v != kRecordingFormatVersion) {
                return Result::fail(Error::make(
                    ErrorCategory::Validation, DeviceType::Combined,
                    "TrajectoryCsvLoader", 3,
                    "录制格式版本不匹配: 需要 " +
                        std::to_string(kRecordingFormatVersion) + " 实际 " +
                        ver));
            }
        }
    }

    // 表头
    std::string header_line;
    if (!std::getline(states, header_line)) return fail_load("states.csv 为空");
    const auto header = csv_split(header_line);
    if (header.size() != CsvStateColumns::count()) {
        return fail_load("表头列数不匹配: 期望 " +
                         std::to_string(CsvStateColumns::count()) + " 实际 " +
                         std::to_string(header.size()));
    }

    // 逐行解析
    std::string line;
    std::vector<TrajectoryPoint> points;
    std::int64_t first_steady_ns = 0;
    bool have_first = false;
    std::size_t row = 0;
    while (std::getline(states, line)) {
        if (line.empty()) continue;
        ++row;
        CombinedRobotState c;
        if (!CsvStateColumns::parse_row(header, line, c)) {
            return fail_load("第 " + std::to_string(row) + " 行解析失败");
        }
        TrajectoryPoint p;
        p.timestamp = c.timestamp;
        const std::int64_t ts_ns = c.timestamp.steady_ns();
        if (!have_first) {
            first_steady_ns = ts_ns;
            have_first = true;
        }
        p.t_offset_ns = ts_ns - first_steady_ns;
        p.arm = c.arm;
        p.hand = c.hand;
        points.push_back(std::move(p));
    }
    if (points.empty()) return fail_load("无有效状态行");

    // 事件关联：每个 event 挂到第一个 t_offset >= (event_steady - 首) 的点
    std::vector<CsvEvent> events;
    if (parse_events(events_path.string(), events)) {
        std::size_t pi = 0;
        for (const auto& e : events) {
            const std::int64_t off = e.steady_ns - first_steady_ns;
            while (pi < points.size() && points[pi].t_offset_ns < off) ++pi;
            if (pi >= points.size()) break;
            if (!points[pi].event.empty()) points[pi].event += ";";
            points[pi].event += e.text;
        }
    }

    // 组装 Trajectory + meta
    out.points = std::move(points);
    TrajectoryMeta& meta = out.meta;
    meta.source_recording = dir.filename().string();  // session_id 目录名
    meta.version = std::to_string(kRecordingFormatVersion);
    meta.sample_count = out.points.size();
    meta.created = out.points.front().timestamp;
    meta.duration_s =
        static_cast<double>(out.points.back().t_offset_ns -
                            out.points.front().t_offset_ns) /
        1e9;
    if (!md.empty()) {
        meta.config_hash = meta_value(md, "config_hash");
        meta.calibration_id = meta_value(md, "calibration_ref");
    }
    return Result::ok();
}

}  // namespace robotics::domain
