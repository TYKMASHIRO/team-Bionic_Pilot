#pragma once

#include <optional>
#include <string>

#include "robotics/domain/errors/Error.hpp"

namespace robotics::infra {

/// RM75 机械臂连接配置
struct RobotConfig {
    std::string ip = "192.168.1.18";
    int port = 8080;
    int sdk_thread_mode = 2;  ///< RM_SINGLE/DUAL/TRIPLE = 0/1/2
    bool auto_reconnect = false;
};

/// O6 灵巧手连接配置
struct HandConfig {
    std::string transport = "rm_passthrough";  ///< rm_passthrough | direct_serial
    std::string serial_device;                  ///< Direct 模式：如 /dev/ttyUSB0
    int baudrate = 115200;                      ///< O6 固定 115200
    int data_bits = 8;
    int stop_bits = 1;
    std::string parity = "none";
    int modbus_slave_id = 0x27;                 ///< 右手
    bool right_hand = true;
    double sample_rate_hz = 50.0;
};

/// 安全配置
struct SafetyConfig {
    double max_joint_velocity_rad_s = 3.0;
    double max_tcp_velocity_m_s = 0.5;
    double max_force_n = 100.0;
    double max_hand_temperature_c = 60.0;
    double max_state_delay_ms = 500.0;
    double start_pose_tolerance_m = 0.02;
    double motion_timeout_s = 10.0;
};

/// 日志配置
struct LoggingConfig {
    std::string level = "info";
    std::string directory = "logs";
    std::string file_name = "robot_hand_control.log";
    std::size_t max_size_bytes = 10ull * 1024 * 1024;
    int max_files = 5;
    bool console = true;
    bool log_vendor_raw = false;
};

/**
 * @brief 配置管理：加载 YAML、类型/范围校验、摘要输出。
 */
class Configuration {
public:
    /// 从文件加载（空路径 = 全部默认值）
    domain::Result load(const std::string& robot_yaml_path,
                        const std::string& safety_yaml_path,
                        const std::string& logging_yaml_path);

    /// 校验（连接设备前调用）
    domain::Result validate() const;

    /// 配置 hash（写入录制元数据）
    std::string config_hash() const;

    const RobotConfig& robot() const { return robot_; }
    const HandConfig& hand() const { return hand_; }
    const SafetyConfig& safety() const { return safety_; }
    const LoggingConfig& logging() const { return logging_; }

    RobotConfig& robot_mut() { return robot_; }
    HandConfig& hand_mut() { return hand_; }
    SafetyConfig& safety_mut() { return safety_; }
    LoggingConfig& logging_mut() { return logging_; }

private:
    RobotConfig robot_;
    HandConfig hand_;
    SafetyConfig safety_;
    LoggingConfig logging_;
    bool loaded_ = false;
};

}  // namespace robotics::infra
