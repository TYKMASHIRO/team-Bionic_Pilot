#include "robotics/infrastructure/config/Configuration.hpp"

#include <sstream>

#include <yaml-cpp/yaml.h>

namespace robotics::infra {

namespace {
// 简单字符串 hash（FNV-1a），用于配置版本追踪
std::string fnv1a(const std::string& s) {
    std::uint64_t hash = 1469598103934665603ull;
    for (unsigned char c : s) {
        hash ^= c;
        hash *= 1099511628211ull;
    }
    std::ostringstream oss;
    oss << std::hex << hash;
    return oss.str();
}
}  // namespace

domain::Result Configuration::load(const std::string& robot_yaml_path,
                                   const std::string& safety_yaml_path,
                                   const std::string& logging_yaml_path) {
    // 一期：YAML 解析失败则返回错误；文件不存在则使用默认值（Mock 友好）。
    auto try_load = [](const std::string& path, auto fn) -> domain::Result {
        if (path.empty()) return domain::Result::ok();
        try {
            YAML::Node root = YAML::LoadFile(path);
            fn(root);
            return domain::Result::ok();
        } catch (const YAML::Exception& e) {
            return domain::Result::fail(domain::Error::make(
                domain::ErrorCategory::Configuration, domain::DeviceType::Unknown,
                "Configuration", 1, std::string("YAML 解析失败: ") + e.what()));
        } catch (const std::exception& e) {
            return domain::Result::fail(domain::Error::make(
                domain::ErrorCategory::Configuration, domain::DeviceType::Unknown,
                "Configuration", 2, std::string("配置文件读取失败: ") + e.what()));
        }
    };

    auto r = try_load(robot_yaml_path, [this](const YAML::Node& n) {
        if (n["arm"] && n["arm"]["ip"]) robot_.ip = n["arm"]["ip"].as<std::string>();
        if (n["arm"] && n["arm"]["port"]) robot_.port = n["arm"]["port"].as<int>();
        if (n["arm"] && n["arm"]["sdk_thread_mode"])
            robot_.sdk_thread_mode = n["arm"]["sdk_thread_mode"].as<int>();
        if (n["hand"] && n["hand"]["transport"])
            hand_.transport = n["hand"]["transport"].as<std::string>();
        if (n["hand"] && n["hand"]["serial_device"])
            hand_.serial_device = n["hand"]["serial_device"].as<std::string>();
        if (n["hand"] && n["hand"]["baudrate"])
            hand_.baudrate = n["hand"]["baudrate"].as<int>();
        if (n["hand"] && n["hand"]["modbus_slave_id"])
            hand_.modbus_slave_id = n["hand"]["modbus_slave_id"].as<int>();
        if (n["hand"] && n["hand"]["right_hand"])
            hand_.right_hand = n["hand"]["right_hand"].as<bool>();
        if (n["hand"] && n["hand"]["sample_rate_hz"])
            hand_.sample_rate_hz = n["hand"]["sample_rate_hz"].as<double>();
    });
    if (!r.success) return r;

    auto s = try_load(safety_yaml_path, [this](const YAML::Node& n) {
        if (n["max_joint_velocity_rad_s"])
            safety_.max_joint_velocity_rad_s = n["max_joint_velocity_rad_s"].as<double>();
        if (n["max_tcp_velocity_m_s"])
            safety_.max_tcp_velocity_m_s = n["max_tcp_velocity_m_s"].as<double>();
        if (n["max_force_n"]) safety_.max_force_n = n["max_force_n"].as<double>();
        if (n["max_hand_temperature_c"])
            safety_.max_hand_temperature_c = n["max_hand_temperature_c"].as<double>();
        if (n["max_state_delay_ms"])
            safety_.max_state_delay_ms = n["max_state_delay_ms"].as<double>();
        if (n["start_pose_tolerance_m"])
            safety_.start_pose_tolerance_m = n["start_pose_tolerance_m"].as<double>();
        if (n["motion_timeout_s"])
            safety_.motion_timeout_s = n["motion_timeout_s"].as<double>();
    });
    if (!s.success) return s;

    auto l = try_load(logging_yaml_path, [this](const YAML::Node& n) {
        if (n["level"]) logging_.level = n["level"].as<std::string>();
        if (n["directory"]) logging_.directory = n["directory"].as<std::string>();
        if (n["console"]) logging_.console = n["console"].as<bool>();
        if (n["log_vendor_raw"]) logging_.log_vendor_raw = n["log_vendor_raw"].as<bool>();
    });
    if (!l.success) return l;

    loaded_ = true;
    return domain::Result::ok();
}

domain::Result Configuration::validate() const {
    if (robot_.port <= 0 || robot_.port > 65535) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Configuration, domain::DeviceType::RobotArm,
            "Configuration", 10, "RM75 端口非法"));
    }
    if (hand_.baudrate != 115200) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Configuration, domain::DeviceType::DexterousHand,
            "Configuration", 11, "O6 波特率固定 115200，不可修改"));
    }
    if (hand_.sample_rate_hz <= 0 || hand_.sample_rate_hz > 1000) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Configuration, domain::DeviceType::DexterousHand,
            "Configuration", 12, "采样频率非法"));
    }
    if (safety_.max_state_delay_ms <= 0) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Configuration, domain::DeviceType::Unknown,
            "Configuration", 13, "状态最大延迟必须 > 0"));
    }
    return domain::Result::ok();
}

std::string Configuration::config_hash() const {
    std::ostringstream oss;
    oss << robot_.ip << ":" << robot_.port << "|" << hand_.transport << "|"
        << hand_.baudrate << "|" << hand_.modbus_slave_id << "|"
        << safety_.max_joint_velocity_rad_s << "|" << safety_.max_force_n;
    return fnv1a(oss.str());
}

}  // namespace robotics::infra
