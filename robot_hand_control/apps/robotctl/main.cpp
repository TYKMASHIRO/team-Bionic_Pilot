#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "drivers/mock/MockDexterousHand.hpp"
#include "drivers/mock/MockRobotArm.hpp"
#include "robotics/infrastructure/config/Configuration.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"
#include "robotics/orchestration/ApplicationService.hpp"
#include "robotics/safety/SafetySupervisor.hpp"
#include "src/infrastructure/time/SystemClock.hpp"
#include "src/services/StateStore.hpp"

// 真实驱动：仅 debug/hardware preset 编译（apps/CMakeLists 注入宏）
#if HAVE_REALMAN_DRIVER
#include "RealManAdapter.hpp"
#endif
#if HAVE_LINKERHAND_DRIVER
#include "LinkerHandAdapter.hpp"
#endif

namespace {

using namespace robotics;

// Ctrl-C/TERM 标志（async-signal-safe：handler 只置位，不调用非安全函数）
volatile std::sig_atomic_t g_stop_requested = 0;
extern "C" void handle_signal(int) { g_stop_requested = 1; }

void print_usage() {
    std::cout <<
        "robotctl - RM75 + O6 控制系统 CLI\n"
        "\n"
        "用法: robotctl [全局选项] <command> [options]\n"
        "\n"
        "全局选项:\n"
        "  --real                使用真实设备（RM75 + O6 经 RM 末端透传）\n"
        "  --enable-motion       显式启用真实运动\n"
        "\n"
        "录制:\n"
        "  record <out_dir> --duration <s> [--rate <hz>]\n"
        "                       连续采集 + 同步录制，到时自动停止（Ctrl-C 提前停止）\n"
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
        "轨迹（阶段5）:\n"
        "  trajectory import <recording_dir>\n"
        "                       导入录制为轨迹资产（data/trajectories/<id>）\n"
        "  trajectory list        列出轨迹\n"
        "  trajectory inspect <id>\n"
        "                       轨迹信息与首尾点预览\n"
        "  trajectory validate <id>\n"
        "                       校验轨迹（时间轴/范围/关节数）\n"
        "  trajectory replay <id> [--speed <x>] [--dry-run] [--enable-motion]\n"
        "                       复现轨迹（--enable-motion 真实运动；Ctrl-C 取消）\n"
        "  [--data-dir <dir>]   轨迹数据目录（默认 data/trajectories）\n"
        "\n"
        "Skill（阶段6）:\n"
        "  skill list            列出已注册 Skill\n"
        "  skill <id> [params] [--dry-run]\n"
        "                       执行 Skill（params 为 JSON 对象；Ctrl-C 取消）\n"
        "                       [--skills-dir <dir>]  Skill manifest 目录（默认 skills/）\n"
        "\n";
}

/// 命令行选项：过滤全局开关后，tokens 保留命令词元与子命令标志
struct CliOptions {
    bool real = false;
    bool enable_motion = false;
    double duration_s = 3.0;
    double rate_hz = 50.0;
    double speed = 1.0;        ///< 轨迹调速倍率（--speed）
    std::string data_dir = "data/trajectories";  ///< 轨迹数据目录（--data-dir）
    std::string skills_dir = "skills";           ///< Skill manifest 目录（--skills-dir）
    std::vector<std::string> tokens;
};

CliOptions parse_args(const std::vector<std::string>& args) {
    CliOptions o;
    auto take_next_double = [&](std::size_t i, double* out) {
        if (i + 1 < args.size()) {
            *out = std::atof(args[i + 1].c_str());
        }
    };
    for (std::size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        if (a == "--real") {
            o.real = true;
        } else if (a == "--enable-motion") {
            o.enable_motion = true;
        } else if (a == "--duration") {
            take_next_double(i, &o.duration_s);
            ++i;
        } else if (a.rfind("--duration=", 0) == 0) {
            o.duration_s = std::atof(a.c_str() + std::strlen("--duration="));
        } else if (a == "--rate") {
            take_next_double(i, &o.rate_hz);
            ++i;
        } else if (a.rfind("--rate=", 0) == 0) {
            o.rate_hz = std::atof(a.c_str() + std::strlen("--rate="));
        } else if (a == "--speed") {
            take_next_double(i, &o.speed);
            ++i;
        } else if (a.rfind("--speed=", 0) == 0) {
            o.speed = std::atof(a.c_str() + std::strlen("--speed="));
        } else if (a == "--data-dir") {
            if (i + 1 < args.size()) {
                o.data_dir = args[i + 1];
                ++i;
            }
        } else if (a.rfind("--data-dir=", 0) == 0) {
            o.data_dir = a.c_str() + std::strlen("--data-dir=");
        } else if (a == "--skills-dir") {
            if (i + 1 < args.size()) {
                o.skills_dir = args[i + 1];
                ++i;
            }
        } else if (a.rfind("--skills-dir=", 0) == 0) {
            o.skills_dir = a.c_str() + std::strlen("--skills-dir=");
        } else {
            o.tokens.push_back(a);
        }
    }
    return o;
}

/// 读文件全文
std::string read_file(const std::string& path) {
    std::ifstream f(path);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

/// 从 key=value 行式文本取 value
std::string extract_value(const std::string& text, const std::string& key) {
    const std::string needle = key + "=";
    const std::size_t pos = text.find(needle);
    if (pos == std::string::npos) return "";
    const std::size_t val = pos + needle.size();
    const std::size_t end = text.find('\n', val);
    return text.substr(val, end == std::string::npos ? std::string::npos : end - val);
}

int run(int argc, char** argv) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    infra::Logger::instance().configure(infra::LogLevel::Info, true, "");
    infra::Configuration config;
    config.robot_mut().ip = "192.168.1.18";
    config.hand_mut().modbus_slave_id = 0x27;

    const std::vector<std::string> args(argv + 1, argv + argc);
    const CliOptions opts = parse_args(args);
    const auto& tokens = opts.tokens;

    if (tokens.empty()) {
        print_usage();
        return 1;
    }
    const std::string cmd = tokens[0];

    auto clock = std::make_shared<infra::SystemClock>();
    auto store = std::make_shared<domain::StateStore>();
    infra::SafetyConfig safety_cfg;
    safety_cfg.max_state_delay_ms = 1000.0;  // Mock 宽松
    auto safety = std::make_shared<domain::SafetySupervisor>(safety_cfg);

    // ---- 设备接线 ----
    // 真实路径：RM75 连接 → native_handle() → O6 经 RM 末端 RS485 透传连接
    // （O6 由 RM75 末端供电：先 arm 后 hand 与供电拓扑一致）。
    std::shared_ptr<domain::IRobotArm> arm;
    std::shared_ptr<domain::IDexterousHand> hand;
    std::shared_ptr<domain::StateCollector> collector;
    std::string device_desc = "mock";

    if (opts.real) {
#if HAVE_REALMAN_DRIVER && HAVE_LINKERHAND_DRIVER
        auto real_arm = std::make_shared<robotics::realman::RealManAdapter>(
            config.robot().ip, config.robot().port, config.robot().sdk_thread_mode);
        const auto r1 = real_arm->connect();
        if (!r1.success) {
            std::cerr << "RM75 连接失败: " << r1.error.to_string() << "\n";
            return 1;
        }
        void* h = real_arm->native_handle();
        if (!h) {
            std::cerr << "RM75 句柄为空（未连接）\n";
            return 1;
        }
        auto real_hand = std::make_shared<robotics::linkerhand::LinkerHandAdapter>(
            h, static_cast<std::uint8_t>(config.hand().modbus_slave_id), 500);
        const auto r2 = real_hand->connect();
        if (!r2.success) {
            std::cerr << "O6 连接失败（RM75 透传）: " << r2.error.to_string() << "\n";
            return 1;
        }
        arm = real_arm;
        hand = real_hand;
        device_desc = "real (RM75+O6)";
        // 真实采集：rate=0 尽速，帧率由 get_state 实际耗时决定
        collector = std::make_shared<domain::StateCollector>(arm, hand, store, clock,
                                                             0.0, 0.0);
#else
        std::cerr << "--real 需要 debug/hardware 构建（驱动未编译）\n";
        return 1;
#endif
    } else {
        arm = std::make_shared<mock::MockRobotArm>();
        hand = std::make_shared<mock::MockDexterousHand>();
    }

    domain::ApplicationService app(clock, arm, hand, store, safety, collector, nullptr);

    if (opts.enable_motion) {
        app.enable_real_motion();
    }

    // ---- 录制（前台循环）----
    if (cmd == "record") {
        if (tokens.size() < 2) {
            print_usage();
            return 1;
        }
        const std::string out_dir = tokens[1];

        auto cr = app.connect_all();
        if (!cr.success) {
            std::cerr << "设备连接失败: " << cr.error.to_string() << "\n";
            return 1;
        }
        if (!app.start_collection().success) {
            std::cerr << "启动采集失败\n";
            return 1;
        }
        const std::string config_hash = config.config_hash();
        auto rs = app.record_start(out_dir, opts.rate_hz, config_hash);
        if (!rs.success) {
            std::cerr << "录制启动失败: " << rs.error.to_string() << "\n";
            app.stop_collection();
            return 1;
        }

        std::cout << "录制中 [" << device_desc << "] session="
                  << app.recording_metadata().session_id
                  << " duration=" << opts.duration_s << "s rate=" << opts.rate_hz
                  << "Hz —— Ctrl-C 可提前停止\n";
        std::cout.flush();

        // 前台轮询：到时或 SIGINT 即停止（handler 只置位，主线程负责清理）
        const auto deadline =
            std::chrono::steady_clock::now() +
            std::chrono::milliseconds(static_cast<std::int64_t>(opts.duration_s * 1000));
        while (std::chrono::steady_clock::now() < deadline && !g_stop_requested) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        const bool early_stop = (g_stop_requested != 0);

        const auto rst = app.record_stop();
        app.stop_collection();
        app.disconnect_all();  // 断开顺序 hand→arm（O6 依赖 RM 供电）

        const auto meta = app.recording_metadata();
        const std::string md =
            read_file(out_dir + "/" + meta.session_id + "/metadata.txt");
        const std::string frames = extract_value(md, "frames_recorded");
        const std::string dropped = extract_value(md, "dropped_frames");

        std::cout << "录制" << (rst.success ? "完成" : "出错")
                  << (early_stop ? "（提前停止，Ctrl-C）" : "") << "\n"
                  << "  session: " << meta.session_id << "\n"
                  << "  目录:    " << out_dir << "/" << meta.session_id << "\n"
                  << "  frames:  " << (frames.empty() ? "?" : frames) << "\n"
                  << "  dropped: " << (dropped.empty() ? "?" : dropped) << "\n"
                  << "  开始:    " << meta.start_time.to_iso8601() << "\n"
                  << "  结束:    " << meta.end_time.to_iso8601() << "\n";
        return rst.success ? 0 : 1;
    }

    auto print_result = [](const domain::Result& r) {
        std::cout << (r.success ? "[ok] " : "[fail] ") << (r.success ? "" : r.error.to_string() + " ") << "\n";
    };

    // ---- 轨迹管理（阶段5）----
    app.set_trajectory_data_dir(opts.data_dir);
    if (cmd == "trajectory") {
        if (tokens.size() < 2) { print_usage(); return 1; }
        const std::string sub = tokens[1];

        if (sub == "import") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            std::string id;
            const auto r = app.trajectory_import(tokens[2], id);
            if (!r.success) { print_result(r); return 1; }
            std::cout << "[ok] 导入轨迹 " << id << " <- " << tokens[2] << "\n";
            return 0;
        }
        if (sub == "list") {
            const auto metas = app.trajectory_list();
            if (metas.empty()) {
                std::cout << "（无轨迹）\n";
                return 0;
            }
            for (const auto& m : metas) {
                std::cout << m.trajectory_id
                          << "  src=" << (m.source_recording.empty() ? "-" : m.source_recording)
                          << "  pts=" << m.sample_count
                          << "  dur=" << m.duration_s << "s"
                          << "  ver=" << (m.version.empty() ? "-" : m.version)
                          << "  created=" << m.created.to_iso8601() << "\n";
            }
            return 0;
        }
        if (sub == "inspect") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            domain::Trajectory traj;
            const auto r = app.trajectory_load(tokens[2], traj);
            if (!r.success) { print_result(r); return 1; }
            const auto& meta = traj.meta;
            std::cout << "id=" << meta.trajectory_id
                      << "  name=" << (meta.name.empty() ? "-" : meta.name)
                      << "  src=" << (meta.source_recording.empty() ? "-" : meta.source_recording)
                      << "  pts=" << meta.sample_count
                      << "  dur=" << meta.duration_s << "s"
                      << "  ver=" << (meta.version.empty() ? "-" : meta.version)
                      << "  calib=" << (meta.calibration_id.empty() ? "-" : meta.calibration_id)
                      << "\n";
            // 首尾点预览
            auto fmt = [](const domain::TrajectoryPoint& p) {
                std::ostringstream ss;
                ss << "t=" << p.t_offset_ns / 1000000 << "ms"
                   << " arm=" << (p.arm.valid ? "ok" : "n/a")
                   << " hand=" << (p.hand.valid ? "ok" : "n/a");
                if (!p.event.empty()) ss << " ev=" << p.event;
                return ss.str();
            };
            if (!traj.points.empty()) {
                std::cout << "  首点: " << fmt(traj.points.front()) << "\n";
                if (traj.points.size() > 1) {
                    std::cout << "  末点: " << fmt(traj.points.back()) << "\n";
                }
            }
            return 0;
        }
        if (sub == "validate") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            domain::TrajectoryValidationReport report;
            const auto r = app.trajectory_validate(tokens[2], report);
            if (!r.success) { print_result(r); return 1; }
            std::cout << (report.valid ? "[ok] " : "[fail] ") << report.to_string() << "\n";
            return report.valid ? 0 : 1;
        }
        if (sub == "replay") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            const bool dry_run =
                std::find(tokens.begin(), tokens.end(), "--dry-run") != tokens.end();

            // 非 dry-run：连接设备 + 真实运动许可
            if (!dry_run) {
                auto cr = app.connect_all();
                if (!cr.success) {
                    std::cerr << "设备连接失败: " << cr.error.to_string() << "\n";
                    return 1;
                }
            }

            domain::ReplayOptions options;
            options.speed = opts.speed;
            options.dry_run = dry_run;
            domain::ReplayReport report;
            const auto cancel = [&]() { return g_stop_requested != 0; };
            const auto r = app.trajectory_replay(tokens[2], options, dry_run,
                                                 cancel, report);
            if (!r.success && report.error.is_ok()) {
                // 前置错误（如未启用运动）没有 report
                std::cerr << "复现失败: " << r.error.to_string() << "\n";
                return 1;
            }
            std::cout << "[replay] " << report.to_string() << "\n";
            return report.success ? 0 : 1;
        }

        print_usage();
        return 1;
    }

    // ---- Skill 框架（阶段6）----
    if (cmd == "skill") {
        // 加载 manifest（目录缺失仅告警，lookup 会自然失败）
        app.load_skills(opts.skills_dir);
        if (tokens.size() < 2) {
            print_usage();
            return 1;
        }
        const std::string sub = tokens[1];
        if (sub == "list") {
            const auto skills = app.skill_list();
            if (skills.empty()) {
                std::cout << "（无已注册 Skill）\n";
                return 0;
            }
            for (const auto& d : skills) {
                std::cout << d.id << "  ver=" << d.version
                          << (d.real_motion ? "  [real-motion]" : "")
                          << "  " << (d.description.empty() ? "" : d.description)
                          << "\n";
            }
            return 0;
        }
        // skill <id> [params_json] [--dry-run]
        const bool dry_run =
            std::find(tokens.begin(), tokens.end(), "--dry-run") != tokens.end();
        std::string params_json;
        for (std::size_t i = 2; i < tokens.size(); ++i) {
            if (tokens[i] == "--dry-run") continue;
            params_json = tokens[i];
            break;
        }
        // 前置需要设备在线（skill preconditions 包含设备在线；Mock 建默认连接）
        {
            const auto cr = app.connect_all();
            if (!cr.success) {
                std::cerr << "设备连接失败: " << cr.error.to_string() << "\n";
                return 1;
            }
        }
        const auto cancel = [&]() { return g_stop_requested != 0; };
        const domain::SkillResult r =
            app.run_skill(sub, params_json, dry_run, cancel);
        std::cout << "[skill] " << r.to_string() << "\n";
        return r.success ? 0 : 1;
    }

    // 其余命令：连接（real 幂等；mock 建立默认连接）
    {
        auto cr = app.connect_all();
        if (!cr.success) {
            std::cerr << "设备连接失败: " << cr.error.to_string() << "\n";
        }
    }

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
        if (tokens.size() < 2) { print_usage(); return 1; }
        const std::string sub = tokens[1];
        if (sub == "connect") { print_result(app.arm_connect()); return 0; }
        if (sub == "status") {
            auto st = app.current_state().arm;
            std::cout << "arm connected=" << arm->is_connected()
                      << " valid=" << st.valid
                      << " motion=" << domain::to_string(st.motion_state) << "\n";
            return 0;
        }
        if (sub == "home") {
            bool dry = std::find(tokens.begin(), tokens.end(), "--dry-run") != tokens.end();
            print_result(app.arm_home(dry));
            return 0;
        }
        if (sub == "stop") { print_result(app.arm_stop()); return 0; }
        if (sub == "drag-teach") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            const std::string op = tokens[2];
            if (op == "start") {
                bool rec = std::find(tokens.begin(), tokens.end(), "--record") != tokens.end();
                print_result(app.arm_drag_teach_start(rec));
                return 0;
            }
            if (op == "stop") { print_result(app.arm_drag_teach_stop()); return 0; }
        }
    }

    // 灵巧手
    if (cmd == "hand") {
        if (tokens.size() < 2) { print_usage(); return 1; }
        const std::string sub = tokens[1];
        if (sub == "connect") { print_result(app.hand_connect()); return 0; }
        if (sub == "status") {
            auto st = app.current_state().hand;
            std::cout << "hand connected=" << hand->is_connected()
                      << " valid=" << st.valid
                      << " state=" << domain::to_string(st.state) << "\n";
            return 0;
        }
        if (sub == "preset") {
            if (tokens.size() < 3) { print_usage(); return 1; }
            const std::string p = tokens[2];
            bool dry = std::find(tokens.begin(), tokens.end(), "--dry-run") != tokens.end();
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
