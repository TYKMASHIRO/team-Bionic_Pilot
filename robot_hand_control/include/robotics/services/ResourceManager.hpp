#pragma once

#include <mutex>
#include <string>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/types/DeviceType.hpp"

namespace robotics::domain {

/**
 * @brief 设备资源互斥。
 * 确保同一设备同一时刻只有一个命令/Skill 拥有执行权。
 */
class ResourceManager {
public:
    /// 尝试获取资源；失败返回错误（ResourceConflict）
    Result acquire(DeviceType device, const std::string& owner);

    /// 释放资源
    void release(DeviceType device, const std::string& owner);

    /// 查询当前占用者
    std::string owner(DeviceType device) const;

private:
    mutable std::mutex mutex_;
    struct Slot {
        bool held = false;
        std::string owner;
    };
    Slot arm_;
    Slot hand_;
};

}  // namespace robotics::domain
