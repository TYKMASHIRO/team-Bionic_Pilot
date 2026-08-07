#include <iomanip>
#include <iostream>
#include <memory>
#include <string>

#include "robotics/infrastructure/config/Configuration.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"

namespace {

using namespace robotics;

int run(int argc, char** argv) {
    // diagnostics: 配置校验、版本信息、环境检查
    infra::Configuration config;
    const std::string robot_yaml =
        (argc > 1) ? argv[1] : std::string();
    auto r = config.load(robot_yaml, "", "");
    if (!r.success) {
        std::cerr << "配置错误: " << r.error.to_string() << "\n";
        return 1;
    }
    auto v = config.validate();
    if (!v.success) {
        std::cerr << "配置校验失败: " << v.error.to_string() << "\n";
        return 1;
    }

    std::cout << "配置摘要:\n";
    std::cout << "  arm: " << config.robot().ip << ":" << config.robot().port
              << "  thread_mode=" << config.robot().sdk_thread_mode << "\n";
    std::cout << "  hand: transport=" << config.hand().transport
              << " baud=" << config.hand().baudrate
              << " slave_id=" << std::hex << config.hand().modbus_slave_id
              << std::dec << "\n";
    std::cout << "  config_hash=" << config.config_hash() << "\n";
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    return run(argc, argv);
}
