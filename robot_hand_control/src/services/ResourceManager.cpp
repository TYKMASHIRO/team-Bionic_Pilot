#include "robotics/services/ResourceManager.hpp"

namespace robotics::domain {

Result ResourceManager::acquire(DeviceType device, const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    Slot* slot = nullptr;
    if (device == DeviceType::RobotArm) slot = &arm_;
    else if (device == DeviceType::DexterousHand) slot = &hand_;
    else if (device == DeviceType::Combined) {
        // 组合资源：需要机械臂和灵巧手都空闲
        if (arm_.held || hand_.held) {
            return Result::fail(Error::make(
                ErrorCategory::ResourceConflict, device, "ResourceManager", 1,
                "组合资源被占用: arm=" + arm_.owner + " hand=" + hand_.owner));
        }
        arm_.held = true; arm_.owner = owner;
        hand_.held = true; hand_.owner = owner;
        return Result::ok();
    } else {
        return Result::fail(Error::make(
            ErrorCategory::Validation, device, "ResourceManager", 2,
            "未知设备资源"));
    }
    if (slot->held) {
        return Result::fail(Error::make(
            ErrorCategory::ResourceConflict, device, "ResourceManager", 1,
            "设备资源已被占用: " + slot->owner));
    }
    slot->held = true;
    slot->owner = owner;
    return Result::ok();
}

void ResourceManager::release(DeviceType device, const std::string& owner) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (device == DeviceType::Combined) {
        if (arm_.owner == owner) { arm_.held = false; arm_.owner.clear(); }
        if (hand_.owner == owner) { hand_.held = false; hand_.owner.clear(); }
        return;
    }
    Slot* slot = (device == DeviceType::RobotArm) ? &arm_
                : (device == DeviceType::DexterousHand) ? &hand_ : nullptr;
    if (slot && slot->owner == owner) {
        slot->held = false;
        slot->owner.clear();
    }
}

std::string ResourceManager::owner(DeviceType device) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const Slot* slot = (device == DeviceType::RobotArm) ? &arm_
                       : (device == DeviceType::DexterousHand) ? &hand_ : nullptr;
    return slot ? slot->owner : std::string();
}

}  // namespace robotics::domain
