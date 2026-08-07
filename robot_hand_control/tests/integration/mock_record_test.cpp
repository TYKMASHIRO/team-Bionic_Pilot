#include <gtest/gtest.h>

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/orchestration/ApplicationService.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

using namespace robotics;

namespace {

std::string read_all(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

// 集成验收：连接 → 采集 → record_start → 事件 → record_stop → 读回落盘
TEST(IntegrationMockRecord, RecordFlowThroughApplicationService) {
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    domain::ApplicationService app(clock, arm, hand, store, safety);

    ASSERT_TRUE(app.connect_all().success);
    ASSERT_TRUE(app.start_collection().success);

    const std::string out_dir = ::testing::TempDir();
    const std::string config_hash = "deadbeef0123";
    ASSERT_TRUE(app.record_start(out_dir, 200.0, config_hash).success);
    EXPECT_TRUE(app.recording());

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    ASSERT_TRUE(app.record_event("grasp-start").success);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    ASSERT_TRUE(app.record_stop().success);
    EXPECT_FALSE(app.recording());

    // 停止后事件/再次录制应报资源冲突而非崩溃
    auto err = app.record_event("x");
    EXPECT_FALSE(err.success);
    EXPECT_EQ(err.error.category, domain::ErrorCategory::ResourceConflict);

    ASSERT_TRUE(app.stop_collection().success);

    // 读回落盘文件断言
    const auto meta = app.recording_metadata();
    const std::string dir = out_dir + "/" + meta.session_id;
    const std::string states = read_all(dir + "/states.csv");
    const std::string events = read_all(dir + "/events.csv");
    const std::string md = read_all(dir + "/metadata.txt");

    int newlines = 0;
    for (char c : states) if (c == '\n') ++newlines;
    EXPECT_GT(newlines, 1);  // 表头 + 至少 1 行状态

    EXPECT_NE(events.find("grasp-start"), std::string::npos);
    EXPECT_NE(md.find("session_id=" + meta.session_id), std::string::npos);
    EXPECT_NE(md.find("config_hash=" + config_hash), std::string::npos);
    EXPECT_NE(md.find("frames_recorded="), std::string::npos);
}
