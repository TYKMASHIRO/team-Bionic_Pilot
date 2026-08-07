#include "robotics/orchestration/ApplicationService.hpp"

#include "robotics/infrastructure/logging/Logger.hpp"

namespace robotics::domain {

namespace {
auto& log() { return robotics::infra::Logger::instance(); }
}  // namespace

ApplicationService::ApplicationService(std::shared_ptr<IClock> clock,
                                       std::shared_ptr<IRobotArm> arm,
                                       std::shared_ptr<IDexterousHand> hand,
                                       std::shared_ptr<IStateStore> store,
                                       std::shared_ptr<ISafetySupervisor> safety)
    : clock_(std::move(clock)),
      arm_(std::move(arm)),
      hand_(std::move(hand)),
      store_(std::move(store)),
      safety_(std::move(safety)) {}

Result ApplicationService::connect_all() {
    if (arm_) {
        auto r = arm_->connect();
        if (!r.success) return r;
        if (store_) store_->update_arm(arm_->get_state());
    }
    if (hand_) {
        auto r = hand_->connect();
        if (!r.success) return r;
        if (store_) store_->update_hand(hand_->get_state());
    }
    return Result::ok();
}

Result ApplicationService::disconnect_all() {
    if (hand_) hand_->disconnect();
    if (arm_) arm_->disconnect();
    return Result::ok();
}

std::vector<DeviceHealth> ApplicationService::diagnose_all() const {
    std::vector<DeviceHealth> result;
    if (arm_) result.push_back(arm_->health_check());
    if (hand_) result.push_back(hand_->health_check());
    return result;
}

CombinedRobotState ApplicationService::current_state() const {
    return store_ ? store_->combined() : CombinedRobotState{};
}

void ApplicationService::enable_real_motion() {
    if (safety_) safety_->set_real_motion_enabled(true);
    log().warn("ApplicationService", "真实运动已启用（--enable-motion）");
}

CommandId ApplicationService::submit_command(Command cmd) {
    // 一期：直接同步执行（阶段4 接入 CommandScheduler 线程）
    return cmd.command_id.empty() ? make_command_id("app") : cmd.command_id;
}

// ---- 机械臂 ----
Result ApplicationService::arm_connect() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 1, "机械臂未注册"));
    auto r = arm_->connect();
    if (r.success && store_) store_->update_arm(arm_->get_state());
    return r;
}

Result ApplicationService::arm_disconnect() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 2, "机械臂未注册"));
    return arm_->disconnect();
}

Result ApplicationService::arm_home(bool dry_run) {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 3, "机械臂未注册"));
    if (dry_run) {
        log().info("ApplicationService", "arm_home --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::RobotArm,
            "ApplicationService", 4, "真实运动未启用"));
    }
    // 安全位（一期用 Mock 语义；真实 home 在阶段2实现）
    return arm_->move_joint(ArmJointVector{0, 0, 0, 0, 0, 0, 0}, 0.2, true);
}

Result ApplicationService::arm_stop() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 5, "机械臂未注册"));
    return arm_->stop();
}

Result ApplicationService::arm_drag_teach_start(bool record) {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 6, "机械臂未注册"));
    return arm_->start_drag_teach(record);
}

Result ApplicationService::arm_drag_teach_stop() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 7, "机械臂未注册"));
    return arm_->stop_drag_teach();
}

// ---- 灵巧手 ----
Result ApplicationService::hand_connect() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 8, "灵巧手未注册"));
    auto r = hand_->connect();
    if (r.success && store_) store_->update_hand(hand_->get_state());
    return r;
}

Result ApplicationService::hand_disconnect() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 9, "灵巧手未注册"));
    return hand_->disconnect();
}

Result ApplicationService::hand_open(bool dry_run) {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 10, "灵巧手未注册"));
    if (dry_run) {
        log().info("ApplicationService", "hand_open --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::DexterousHand,
            "ApplicationService", 11, "真实运动未启用"));
    }
    return hand_->apply_preset(HandPreset::Open);
}

Result ApplicationService::hand_close(bool dry_run) {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 12, "灵巧手未注册"));
    if (dry_run) {
        log().info("ApplicationService", "hand_close --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::DexterousHand,
            "ApplicationService", 13, "真实运动未启用"));
    }
    return hand_->apply_preset(HandPreset::Close);
}

Result ApplicationService::hand_stop() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 14, "灵巧手未注册"));
    return hand_->stop();
}

SkillResult ApplicationService::run_skill(const std::string& skill_id,
                                         const std::string& parameters_json,
                                         bool dry_run) {
    (void)parameters_json;
    (void)dry_run;
    SkillResult result;
    result.skill_id = skill_id;
    result.skill_version = "0.0.0";
    // 阶段6 实现 SkillRuntime 后接入真实执行
    result.failed_stage = "skill_runtime";
    result.error = Error::make(ErrorCategory::Unsupported, DeviceType::Combined,
        "ApplicationService", 15,
        "Skill 运行时未实现（阶段6）");
    result.success = false;
    result.final_state = CommandState::Failed;
    return result;
}

}  // namespace robotics::domain
