#pragma once

#include <string>

namespace robotics::domain {

/// 设备类型
enum class DeviceType {
    Unknown = 0,
    RobotArm,     ///< 机械臂（RM75-6F）
    DexterousHand, ///< 灵巧手（O6）
    Combined,     ///< 组合
};

/// 设备状态枚举（生命周期）
enum class DeviceConnectionState {
    Disconnected = 0,
    Connecting,
    Connected,
    Error,
};

/// 运动状态（机械臂）
enum class ArmMotionState {
    Unknown = 0,
    Idle,           ///< 空闲
    Moving,         ///< 运动中
    Paused,         ///< 暂停
    Reaching,       ///< 到位中
    Stopped,        ///< 已停止（运动被停止）
    EmergencyStopped, ///< 急停
    DragTeaching,   ///< 拖动示教中
    Replaying,      ///< 轨迹复现中
};

/// 灵巧手状态
enum class HandState {
    Unknown = 0,
    Idle,
    Moving,
    Stalled,   ///< 堵转/卡住
    Faulted,   ///< 故障
};

std::string to_string(DeviceType type);
std::string to_string(DeviceConnectionState state);
std::string to_string(ArmMotionState state);
std::string to_string(HandState state);

}  // namespace robotics::domain
