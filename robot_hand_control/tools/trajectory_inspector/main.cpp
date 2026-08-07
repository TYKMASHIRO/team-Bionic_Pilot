// trajectory_inspector：检查录制目录/轨迹资产目录能否加载为轨迹并给出校验报告。
//
// 用法:
//   trajectory_inspector <dir>            加载 + 打印元数据 + 首尾点
//   trajectory_inspector <dir> --validate 加载 + 校验报告（退出码=是否通过）
#include <iostream>
#include <sstream>
#include <string>

#include "robotics/trajectory/TrajectoryValidator.hpp"
#include "src/trajectory/TrajectoryCsvLoader.hpp"

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "用法: trajectory_inspector <dir> [--validate]\n";
        return 2;
    }
    const std::string dir = argv[1];
    bool do_validate = false;
    for (int i = 2; i < argc; ++i) {
        if (std::string(argv[i]) == "--validate") do_validate = true;
    }

    robotics::domain::Trajectory traj;
    const auto r = robotics::domain::TrajectoryCsvLoader::load_recording(dir, traj);
    if (!r.success) {
        std::cerr << "加载失败: " << r.error.to_string() << "\n";
        return 1;
    }

    const auto& meta = traj.meta;
    std::cout << "source=" << meta.source_recording
              << "  pts=" << meta.sample_count
              << "  dur=" << meta.duration_s << "s"
              << "  ver=" << meta.version
              << "  calib=" << (meta.calibration_id.empty() ? "-" : meta.calibration_id)
              << "\n";

    auto fmt = [](const robotics::domain::TrajectoryPoint& p) {
        std::ostringstream ss;
        ss << "t=" << p.t_offset_ns / 1000000 << "ms"
           << " arm=" << (p.arm.valid ? "ok" : "n/a")
           << " hand=" << (p.hand.valid ? "ok" : "n/a");
        if (!p.event.empty()) ss << " ev=" << p.event;
        return ss.str();
    };
    if (!traj.points.empty()) {
        std::cout << "首点: " << fmt(traj.points.front()) << "\n";
        if (traj.points.size() > 1) {
            std::cout << "末点: " << fmt(traj.points.back()) << "\n";
        }
    }

    if (do_validate) {
        robotics::domain::TrajectoryValidationReport report;
        robotics::domain::TrajectoryValidator::validate(traj, report);
        std::cout << (report.valid ? "[ok] " : "[fail] ") << report.to_string() << "\n";
        return report.valid ? 0 : 1;
    }
    return 0;
}
