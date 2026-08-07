#pragma once

#include <string>

#include "robotics/domain/types/DeviceType.hpp"

namespace robotics::domain {

/// 设备健康状态
enum class HealthLevel {
    Unknown = 0,
    Ok,            ///< 健康
    Warning,       ///< 警告（如温度偏高）
    Degraded,      ///< 降级（部分功能不可用）
    Fault,         ///< 故障
};

/**
 * @brief 设备健康检查结果。
 */
struct DeviceHealth {
    DeviceType device = DeviceType::Unknown;
    HealthLevel level = HealthLevel::Unknown;
    std::string summary;    ///< 人类可读摘要
    std::string sdk_version;
    std::string firmware_version;
    bool online = false;
    bool driver_loaded = false;
    std::string detail;     ///< 详细说明/错误
};

}  // namespace robotics::domain
