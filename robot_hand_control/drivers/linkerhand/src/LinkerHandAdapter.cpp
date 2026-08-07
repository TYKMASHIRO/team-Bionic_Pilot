// LinkerHandAdapter —— 阶段3 完整实现。
// 当前为骨架：所有接口返回 Unsupported，保证工程可编译。
#include "drivers/linkerhand/include/LinkerHandAdapter.hpp"

namespace robotics::linkerhand {

using namespace robotics::domain;

LinkerHandAdapter::LinkerHandAdapter() = default;
LinkerHandAdapter::~LinkerHandAdapter() {
    disconnect();
}

Result LinkerHandAdapter::connect() {
    return Result::fail(Error::make(ErrorCategory::Unsupported, DeviceType::DexterousHand,
        "LinkerHandAdapter", 1, "O6 驱动阶段3实现"));
}
Result LinkerHandAdapter::disconnect() { return Result::ok(); }
bool LinkerHandAdapter::is_connected() const { return false; }
DexterousHandState LinkerHandAdapter::get_state() const { return DexterousHandState{}; }

Result LinkerHandAdapter::set_joint_positions(const HandJointVector&) {
    return unsupported();
}
Result LinkerHandAdapter::set_joint_speeds(const HandJointVector&) {
    return unsupported();
}
Result LinkerHandAdapter::set_torque_limits(const HandJointVector&) {
    return unsupported();
}
Result LinkerHandAdapter::apply_preset(HandPreset) { return unsupported(); }
Result LinkerHandAdapter::stop() { return unsupported(); }
Result LinkerHandAdapter::clear_error() { return unsupported(); }
DeviceHealth LinkerHandAdapter::health_check() {
    DeviceHealth h;
    h.device = DeviceType::DexterousHand;
    h.driver_loaded = true;
    h.sdk_version = "2.0.0";
    h.level = HealthLevel::Fault;
    h.summary = "LinkerHand driver not implemented (stage 3)";
    return h;
}

Result LinkerHandAdapter::unsupported() const {
    return Result::fail(Error::make(ErrorCategory::Unsupported, DeviceType::DexterousHand,
        "LinkerHandAdapter", 99, "该能力阶段3实现"));
}

}  // namespace robotics::linkerhand
