#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/infrastructure/config/Configuration.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"
#include "robotics/orchestration/ApplicationService.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

namespace {

using namespace robotics;

void print_usage() {
    std::cout <<
        "robotctl - RM75 + O6 控制系统 CLI\n"
        "\n"
        "用法: robotctl <command> [options]\n"
        "\n"
        "诊断:\n"
        "  doctor                 设备健康诊断\n"
        "  status                 组合状态\n"
        "\n"
        "机械臂:\n"
        "  arm connect\n"
        "  arm status\n"
        "  arm home [--dry-run]\n"
        "  arm stop\n"
        "  arm drag-teach start [--record]\n"
        "  arm drag-teach stop\n"
        "\n"
        "灵巧手:\n"
        "  hand connect\n"
        "  hand status\n"
        "  hand preset open [--dry-run]\n"
        "  hand preset close [--dry-run]\n"
        "  hand stop\n"
        "\n"
        "安全:\n"
        "  --enable-motion        显式启用真实运动\n"
        "\n";
}

int run(int argc, char** argv) {
    // 默认使用 Mock 设备（阶段1）；真实设备在阶段2/3接入
    auto clock = std::make_shared<infra::SystemClock>();
    auto arm = std::make_shared<mock::MockRobotArm>();
    auto hand = std::make_shared<mock::MockDexterousHand>();
    auto store = std::make_shared<domain::StateStore>();

    infra::Configuration config;
    infra::SafetyConfig safety_cfg;
    safety_cfg.max_state_delay_ms = 1000.0;  // Mock 宽松
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    infra::Logger::instance().configure(infra::LogLevel::Info, true, "");
    config.robot_mut().ip = "192.168.1.18";
    config.hand_mut().modbus_slave_id = 0x27;

    domain::ApplicationService app(clock, arm, hand, store, safety);

    const std::vector<std::string> args(argv + 1, argv + argc);

    // 全局安全选项
    bool enable_motion = false;
    for (const auto& a : args) {
        if (a == "--enable-motion") enable_motion = true;
    }
    if (enable_motion) {
        app.enable_real_motion();
    }

    // 阶段1：默认连接 Mock 设备（CLI 进程隔离，每次命令独立进程）。
    // 真实驱动（阶段2/3）接入后，改为按命令显式连接。
    {
        auto r = app.connect_all();
        if (!r.success) {
            std::cerr << "设备连接失败: " << r.error.to_string() << "\n";
        }
    }

    auto print_result = [](const domain::Result& r) {
        std::cout << (r.success ? "[ok] " : "[fail] ") << (r.success ? "" : r.error.to_string() + " ") << "\n";
    };

    if (args.empty()) {
        print_usage();
        return 1;
    }

    const std::string cmd = args[0];

    // 诊断
    if (cmd == "doctor") {
        for (const auto& h : app.diagnose_all()) {
            std::cout << "- " << (h.device == domain::DeviceType::RobotArm ? "arm" : "hand")
                      << ": " << (h.online ? "online" : "offline")
                      << " | " << h.summary << "\n";
        }
        return 0;
    }
    if (cmd == "status") {
        auto st = app.current_state();
        std::cout << "arm:  " << (st.arm_present() ? "present" : "absent")
                  << " motion=" << domain::to_string(st.arm.motion_state) << "\n";
        std::cout << "hand: " << (st.hand_present() ? "present" : "absent")
                  << " state=" << domain::to_string(st.hand.state) << "\n";
        if (st.arm_present() && st.hand_present()) {
            std::cout << "sync: delta_ns=" << st.time_delta_ns
                      << " quality=" << (st.sync_quality == domain::SyncQuality::Good ? "good"
                                        : st.sync_quality == domain::SyncQuality::Skewed ? "skewed" : "lost") << "\n";
        }
        return 0;
    }

    // 机械臂
    if (cmd == "arm") {
        if (args.size() < 2) { print_usage(); return 1; }
        const std::string sub = args[1];
        if (sub == "connect") { print_result(app.arm_connect()); return 0; }
        if (sub == "status") {
            auto st = app.current_state().arm;
            std::cout << "arm connected=" << arm->is_connected()
                      << " valid=" << st.valid
                      << " motion=" << domain::to_string(st.motion_state) << "\n";
            return 0;
        }
        if (sub == "home") {
            bool dry = std::find(args.begin(), args.end(), "--dry-run") != args.end();
            print_result(app.arm_home(dry));
            return 0;
        }
        if (sub == "stop") { print_result(app.arm_stop()); return 0; }
        if (sub == "drag-teach") {
            if (args.size() < 3) { print_usage(); return 1; }
            const std::string op = args[2];
            if (op == "start") {
                bool rec = std::find(args.begin(), args.end(), "--record") != args.end();
                print_result(app.arm_drag_teach_start(rec));
                return 0;
            }
            if (op == "stop") { print_result(app.arm_drag_teach_stop()); return 0; }
        }
    }

    // 灵巧手
    if (cmd == "hand") {
        if (args.size() < 2) { print_usage(); return 1; }
        const std::string sub = args[1];
        if (sub == "connect") { print_result(app.hand_connect()); return 0; }
        if (sub == "status") {
            auto st = app.current_state().hand;
            std::cout << "hand connected=" << hand->is_connected()
                      << " valid=" << st.valid
                      << " state=" << domain::to_string(st.state) << "\n";
            return 0;
        }
        if (sub == "preset") {
            if (args.size() < 3) { print_usage(); return 1; }
            const std::string p = args[2];
            bool dry = std::find(args.begin(), args.end(), "--dry-run") != args.end();
            if (p == "open") { print_result(app.hand_open(dry)); return 0; }
            if (p == "close") { print_result(app.hand_close(dry)); return 0; }
        }
        if (sub == "stop") { print_result(app.hand_stop()); return 0; }
    }

    print_usage();
    return 1;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "fatal: " << e.what() << "\n";
        return 1;
    }
}
