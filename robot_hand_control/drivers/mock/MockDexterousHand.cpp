#include "drivers/mock/MockDexterousHand.hpp"

namespace robotics::mock {

using namespace robotics::domain;

Result MockDexterousHand::connect() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (fault_) {
        return Result::fail(Error::make(ErrorCategory::Communication,
                                        DeviceType::DexterousHand,
                                        "MockDexterousHand", 1, "注入通信故障"));
    }
    connected_ = true;
    // 默认张开
    position_.fill(255.0);
    state_ = HandState::Idle;
    return Result::ok();
}

Result MockDexterousHand::disconnect() {
    std::lock_guard<std::mutex> lock(mutex_);
    connected_ = false;
    return Result::ok();
}

bool MockDexterousHand::is_connected() const {
    return connected_;
}

DexterousHandState MockDexterousHand::get_state() const {
    std::lock_guard<std::mutex> lock(mutex_);
    DexterousHandState st;
    st.position = position_;
    st.velocity = speed_;
    st.torque = torque_;
    st.temperature.fill(sim_temperature_);
    st.state = state_;
    st.valid = connected_;
    st.fresh = connected_;
    st.timestamp = make_timestamp();
    st.sequence = 0;  // 阶段4 由 StateStore 统一打序号
    return st;
}

Result MockDexterousHand::set_joint_positions(const HandJointVector& positions) {
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::DexterousHand, "MockDexterousHand", 2,
        "未连接"));
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = positions;
    state_ = HandState::Idle;
    return Result::ok();
}

Result MockDexterousHand::set_joint_speeds(const HandJointVector& speeds) {
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::DexterousHand, "MockDexterousHand", 2,
        "未连接"));
    std::lock_guard<std::mutex> lock(mutex_);
    speed_ = speeds;
    return Result::ok();
}

Result MockDexterousHand::set_torque_limits(const HandJointVector& torques) {
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::DexterousHand, "MockDexterousHand", 2,
        "未连接"));
    std::lock_guard<std::mutex> lock(mutex_);
    torque_ = torques;
    return Result::ok();
}

Result MockDexterousHand::apply_preset(HandPreset preset) {
    if (!connected_) return Result::fail(Error::make(
        ErrorCategory::NotConnected, DeviceType::DexterousHand, "MockDexterousHand", 2,
        "未连接"));
    std::lock_guard<std::mutex> lock(mutex_);
    switch (preset) {
        case HandPreset::Open:
            position_ = {255, 255, 255, 255, 255, 255};
            break;
        case HandPreset::Close:
            position_ = {0, 0, 0, 0, 0, 0};
            break;
        case HandPreset::PreGrasp:
            position_ = {255, 128, 255, 255, 255, 255};
            break;
        case HandPreset::Custom:
            break;
    }
    state_ = HandState::Idle;
    return Result::ok();
}

Result MockDexterousHand::stop() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = HandState::Idle;
    return Result::ok();
}

Result MockDexterousHand::clear_error() {
    fault_ = false;
    return Result::ok();
}

DeviceHealth MockDexterousHand::health_check() {
    DeviceHealth h;
    h.device = DeviceType::DexterousHand;
    h.online = connected_;
    h.driver_loaded = true;
    h.sdk_version = "mock";
    h.firmware_version = "mock";
    h.level = (connected_ && !fault_) ? HealthLevel::Ok : HealthLevel::Fault;
    h.summary = connected_ ? "mock hand ok" : "mock hand disconnected";
    return h;
}

}  // namespace robotics::mock
