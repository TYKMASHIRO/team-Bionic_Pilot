// config_validator：校验配置文件（robot.yaml / safety.yaml / logging.yaml）
#include <iostream>
#include <string>

#include "robotics/infrastructure/config/Configuration.hpp"

int main(int argc, char** argv) {
    if (argc < 4) {
        std::cerr << "用法: config_validator <robot.yaml> <safety.yaml> <logging.yaml>\n";
        return 2;
    }
    robotics::infra::Configuration config;
    auto r = config.load(argv[1], argv[2], argv[3]);
    if (!r.success) {
        std::cerr << "加载失败: " << r.error.to_string() << "\n";
        return 1;
    }
    auto v = config.validate();
    if (!v.success) {
        std::cerr << "校验失败: " << v.error.to_string() << "\n";
        return 1;
    }
    std::cout << "配置有效. hash=" << config.config_hash() << "\n";
    return 0;
}
