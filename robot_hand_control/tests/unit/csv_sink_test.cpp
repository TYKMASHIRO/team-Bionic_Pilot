#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "robotics/services/RecordingMetadata.hpp"
#include "src/infrastructure/recording/CsvRecordSink.hpp"

using namespace robotics::domain;
using robotics::infra::CsvRecordSink;

namespace {

// 简单 CSV 行解析（测试数据不含带逗号的未转义值；支持双引号转义）。
std::vector<std::string> split_csv_line(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    bool in_quotes = false;
    for (char ch : line) {
        if (ch == '"') {
            in_quotes = !in_quotes;
        } else if (ch == ',' && !in_quotes) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += ch;
        }
    }
    out.push_back(cur);
    return out;
}

std::vector<std::string> read_lines(const std::string& path) {
    std::ifstream f(path);
    std::vector<std::string> lines;
    std::string line;
    while (std::getline(f, line)) {
        lines.push_back(line);
    }
    return lines;
}

std::string read_all(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

CombinedRobotState make_state(std::uint64_t seq) {
    CombinedRobotState c;
    c.sequence = seq;
    c.timestamp.steady = std::chrono::steady_clock::time_point(
        std::chrono::nanoseconds(static_cast<std::int64_t>(seq) * 10));
    c.timestamp.wall = std::chrono::system_clock::now();
    c.sync_quality = SyncQuality::Good;
    c.time_delta_ns = 1'000'000;

    c.arm.valid = true;
    c.arm.fresh = true;
    c.arm.sequence = seq;
    c.arm.timestamp = c.timestamp;
    c.arm.joint_state.valid = true;
    c.arm.joint_state.position = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
    c.arm.joint_state.temperature = {30, 31, 32, 33, 34, 35, 36};
    c.arm.tcp_pose.position = {0.5, 0.1, 0.2};
    c.arm.force_torque.force = {1.0, 2.0, 3.0};
    c.arm.system_errors = {};

    c.hand.valid = true;
    c.hand.fresh = true;
    c.hand.sequence = seq;
    c.hand.timestamp = c.timestamp;
    c.hand.position = {255, 128, 255, 0, 64, 200};
    return c;
}

}  // namespace

TEST(CsvSinkTest, HeaderAndRowsHave110Columns) {
    RecordingMetadata meta;
    meta.session_id = "rec_test_cols";
    meta.config_hash = "abc";
    meta.sample_rate_hz = 50.0;
    meta.start_time = make_timestamp();

    CsvRecordSink sink(::testing::TempDir(), meta);
    ASSERT_TRUE(sink.is_open());
    for (std::uint64_t i = 1; i <= 3; ++i) {
        ASSERT_TRUE(sink.write_state(make_state(i)));
    }
    ASSERT_TRUE(sink.flush());

    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    const auto lines = read_lines(dir + "/states.csv");
    ASSERT_GE(lines.size(), 4u);  // 表头 + 3 行
    const auto header = split_csv_line(lines[0]);
    ASSERT_EQ(header.size(), 110u);
    for (std::size_t i = 1; i < lines.size(); ++i) {
        ASSERT_EQ(split_csv_line(lines[i]).size(), 110u)
            << "row " << i << " column count mismatch";
    }
}

TEST(CsvSinkTest, SeqAndSteadyMonotonic) {
    RecordingMetadata meta;
    meta.session_id = "rec_test_mono";
    meta.start_time = make_timestamp();
    CsvRecordSink sink(::testing::TempDir(), meta);
    ASSERT_TRUE(sink.is_open());
    for (std::uint64_t i = 1; i <= 5; ++i) {
        ASSERT_TRUE(sink.write_state(make_state(i)));
    }
    ASSERT_TRUE(sink.flush());

    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    const auto lines = read_lines(dir + "/states.csv");
    std::uint64_t prev_seq = 0;
    std::int64_t prev_steady = -1;
    for (std::size_t i = 1; i < lines.size(); ++i) {
        const auto cols = split_csv_line(lines[i]);
        const std::uint64_t seq = std::stoull(cols[0]);
        const std::int64_t steady = std::stoll(cols[1]);
        EXPECT_GT(seq, prev_seq);
        EXPECT_GT(steady, prev_steady);
        prev_seq = seq;
        prev_steady = steady;
    }
}

TEST(CsvSinkTest, EventsAndMetadataWritten) {
    RecordingMetadata meta;
    meta.session_id = "rec_test_events";
    meta.config_hash = "hash123";
    meta.start_time = make_timestamp();
    CsvRecordSink sink(::testing::TempDir(), meta);
    ASSERT_TRUE(sink.is_open());
    ASSERT_TRUE(sink.write_state(make_state(1)));
    ASSERT_TRUE(sink.write_event(make_timestamp(), "teach-start"));
    ASSERT_TRUE(sink.write_command("{\"cmd\":\"home\"}"));
    sink.add_dropped_frames(3);
    ASSERT_TRUE(sink.flush());

    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    const auto ev_lines = read_lines(dir + "/events.csv");
    ASSERT_GE(ev_lines.size(), 1u);
    ASSERT_TRUE(ev_lines[0].find("teach-start") != std::string::npos);

    const auto cmd_lines = read_lines(dir + "/commands.csv");
    ASSERT_FALSE(cmd_lines.empty());
    ASSERT_TRUE(cmd_lines[0].find("home") != std::string::npos);

    const std::string md = read_all(dir + "/metadata.txt");
    EXPECT_NE(md.find("session_id=rec_test_events"), std::string::npos);
    EXPECT_NE(md.find("config_hash=hash123"), std::string::npos);
    EXPECT_NE(md.find("dropped_frames=3"), std::string::npos);
    EXPECT_NE(md.find("frames_recorded=1"), std::string::npos);
}

TEST(CsvSinkTest, AbsentDeviceEncoding) {
    RecordingMetadata meta;
    meta.session_id = "rec_test_absent";
    meta.start_time = make_timestamp();
    CsvRecordSink sink(::testing::TempDir(), meta);
    ASSERT_TRUE(sink.is_open());

    auto c = make_state(1);
    c.hand.valid = false;   // 手缺席
    c.hand.position = {255, 255, 255, 255, 255, 255};
    c.arm.system_errors = {7, 9};
    ASSERT_TRUE(sink.write_state(c));
    ASSERT_TRUE(sink.flush());

    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    const auto lines = read_lines(dir + "/states.csv");
    ASSERT_GE(lines.size(), 2u);
    const auto cols = split_csv_line(lines[1]);

    // hand_valid 列 = "0"；hand_pos_0 列 = "0"；hand_pressure 空串
    EXPECT_EQ(cols[71], "0");           // hand_valid
    EXPECT_EQ(cols[76], "0");           // hand_pos_0（absent → 数值 0）
    EXPECT_EQ(cols[107], "");           // hand_pressure_shape
    EXPECT_EQ(cols[109], "");           // hand_pressure（最后列）
    // arm 正常：arm_sys_err 列 = "7;9"
    EXPECT_EQ(cols[70], "7;9");
}

TEST(CsvSinkTest, SessionDirCreatedAndIsOpenFalseOnBadDir) {
    RecordingMetadata meta;
    meta.session_id = "rec_test_dir";
    meta.start_time = make_timestamp();
    CsvRecordSink sink(::testing::TempDir(), meta);
    ASSERT_TRUE(sink.is_open());
    ASSERT_TRUE(sink.flush());  // 使 metadata 静态块落盘可读
    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    EXPECT_TRUE(read_all(dir + "/metadata.txt").find("session_id=rec_test_dir") !=
                std::string::npos);
    EXPECT_TRUE(read_all(dir + "/metadata.txt").find("start_iso=") !=
                std::string::npos);
}
