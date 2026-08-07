#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <fstream>
#include <memory>
#include <sstream>
#include <thread>

#include "robotics/interfaces/IRecordSink.hpp"
#include "robotics/services/Recorder.hpp"
#include "src/infrastructure/recording/CsvRecordSink.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics::domain;
using robotics::infra::CsvRecordSink;
using robotics::infra::SystemClock;

namespace {

/// 写盘阻塞的 TestSink：模拟慢磁盘，制造队列溢出
class BlockingSink : public IRecordSink {
public:
    bool write_state(const CombinedRobotState&) override {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        states_.fetch_add(1);
        return true;
    }
    bool write_event(Timestamp, const std::string&) override { return true; }
    bool write_command(const std::string&) override { return true; }
    bool flush() override { return true; }
    void add_dropped_frames(std::size_t) override {}
    std::size_t dropped_frames() const override { return 0; }
    std::size_t states() const { return states_.load(); }

private:
    std::atomic<std::size_t> states_{0};
};

std::string read_all(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

// 正常流程：持续采集写盘、事件入队、受控停止后事件与状态都已落盘
TEST(RecorderTest, RecordsStatesAndEventsThenFlushes) {
    auto store = std::make_shared<StateStore>();
    auto clock = std::make_shared<SystemClock>();
    RecordingMetadata meta;
    meta.session_id = "rec_recorder_flow";
    meta.start_time = make_timestamp();
    auto sink = std::make_shared<CsvRecordSink>(::testing::TempDir(), meta);
    ASSERT_TRUE(sink->is_open());

    Recorder recorder(store, clock, sink, 200.0);

    std::atomic<bool> feed{true};
    std::thread feeder([&] {
        RobotArmState a;
        a.valid = true;
        DexterousHandState h;
        h.valid = true;
        while (feed.load()) {
            store->update_arm(a);
            store->update_hand(h);
        }
    });

    ASSERT_TRUE(recorder.start().success);
    EXPECT_TRUE(recorder.is_recording());
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    recorder.record_event("teach-start");
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
    ASSERT_TRUE(recorder.stop().success);
    EXPECT_FALSE(recorder.is_recording());
    feed.store(false);
    feeder.join();

    EXPECT_GT(sink->rows_written(), 0u);
    const std::string dir = std::string(::testing::TempDir()) + "/" + meta.session_id;
    EXPECT_NE(read_all(dir + "/events.csv").find("teach-start"), std::string::npos);
    // stop() 已 flush：metadata 带汇总块
    EXPECT_NE(read_all(dir + "/metadata.txt").find("frames_recorded="),
              std::string::npos);
}

// 序号去重：store 未更新时，反复采样不写重复行
TEST(RecorderTest, DeduplicatesUnchangedFrames) {
    auto store = std::make_shared<StateStore>();
    auto clock = std::make_shared<SystemClock>();
    RobotArmState a;
    a.valid = true;
    DexterousHandState h;
    h.valid = true;
    store->update_arm(a);
    store->update_hand(h);

    RecordingMetadata meta;
    meta.session_id = "rec_recorder_dedup";
    meta.start_time = make_timestamp();
    auto sink = std::make_shared<CsvRecordSink>(::testing::TempDir(), meta);
    ASSERT_TRUE(sink->is_open());

    Recorder recorder(store, clock, sink, 1000.0);
    ASSERT_TRUE(recorder.start().success);
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
    ASSERT_TRUE(recorder.stop().success);

    EXPECT_EQ(sink->rows_written(), 1u);  // 首帧落盘，其后全部去重
}

// 队列溢出：慢写盘 + 小队列 + 尽速采样 → 丢帧计数 > 0，且不阻塞采样
TEST(RecorderTest, DropsFramesWhenQueueFull) {
    auto store = std::make_shared<StateStore>();
    auto clock = std::make_shared<SystemClock>();
    auto sink = std::make_shared<BlockingSink>();
    Recorder recorder(store, clock, sink, 0.0, /*max_queue=*/2);

    std::atomic<bool> feed{true};
    std::thread feeder([&] {
        RobotArmState a;
        a.valid = true;
        while (feed.load()) store->update_arm(a);  // 高速改变序号
    });

    ASSERT_TRUE(recorder.start().success);
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    ASSERT_TRUE(recorder.stop().success);
    feed.store(false);
    feeder.join();

    EXPECT_GT(recorder.dropped_frames(), 0u);  // 队列满丢新帧
    EXPECT_GT(sink->states(), 0u);             // 仍有帧被写出
}

// stop 幂等；析构自动 stop 不崩溃
TEST(RecorderTest, StopIdempotentAndDestructor) {
    auto store = std::make_shared<StateStore>();
    auto clock = std::make_shared<SystemClock>();
    auto sink = std::make_shared<BlockingSink>();
    {
        Recorder recorder(store, clock, sink, 0.0, 4);
        ASSERT_TRUE(recorder.start().success);
        recorder.record_event("x");
        ASSERT_TRUE(recorder.stop().success);
        ASSERT_TRUE(recorder.stop().success);  // 幂等
    }  // 析构自动 stop（已停止则直接返回）
}
