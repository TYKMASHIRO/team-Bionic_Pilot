#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

SkillBase::SkillBase(domain::SkillDescriptor desc, SkillContext ctx)
    : desc_(std::move(desc)), ctx_(std::move(ctx)) {}

domain::Result SkillBase::validate(const domain::SkillParams& params) {
    return check_preconditions(params);
}

domain::SkillResult SkillBase::execute(const domain::SkillParams& params) {
    domain::SkillResult r = make_base_result();

    const domain::Result pre = check_preconditions(params);
    if (!pre.success) {
        return make_fail("validate", pre.error);
    }
    if (params.dry_run) {
        // 只校验不运动（已通过预置条件）
        r.success = true;
        r.failed_stage = "dry_run";
        r.final_state = domain::CommandState::Succeeded;
        r.final_device_summary = device_summary();
        return r;
    }
    if (desc_.real_motion) {
        const domain::Result g = require_real_motion();
        if (!g.success) {
            return make_fail("safety", g.error);
        }
    }
    return run(params);
}

// ---- 辅助 ----

domain::Result SkillBase::require_arm() const {
    if (!ctx_.arm) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::NotConnected, domain::DeviceType::RobotArm,
            "Skill", 50, "机械臂未注册"));
    }
    return domain::Result::ok();
}

domain::Result SkillBase::require_hand() const {
    if (!ctx_.hand) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::NotConnected, domain::DeviceType::DexterousHand,
            "Skill", 51, "灵巧手未注册"));
    }
    return domain::Result::ok();
}

domain::Result SkillBase::require_connected_arm() const {
    const domain::Result r = require_arm();
    if (!r.success) return r;
    if (!ctx_.arm->is_connected()) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::NotConnected, domain::DeviceType::RobotArm,
            "Skill", 52, "机械臂未连接"));
    }
    return domain::Result::ok();
}

domain::Result SkillBase::require_connected_hand() const {
    const domain::Result r = require_hand();
    if (!r.success) return r;
    if (!ctx_.hand->is_connected()) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::NotConnected, domain::DeviceType::DexterousHand,
            "Skill", 53, "灵巧手未连接"));
    }
    return domain::Result::ok();
}

domain::Result SkillBase::require_real_motion() const {
    if (ctx_.safety && !ctx_.safety->real_motion_enabled()) {
        return domain::Result::fail(domain::Error::make(
            domain::ErrorCategory::Safety, domain::DeviceType::Combined, "Skill",
            54, "真实运动未启用（需要 --enable-motion 显式许可）"));
    }
    return domain::Result::ok();
}

domain::SkillResult SkillBase::make_base_result() const {
    domain::SkillResult r;
    r.skill_id = desc_.id;
    r.skill_version = desc_.version;
    r.config_version = "default";  // 一期：由上层注入 config hash 后覆盖
    return r;
}

domain::SkillResult SkillBase::make_ok(bool safety_stopped) const {
    domain::SkillResult r = make_base_result();
    r.success = true;
    r.final_state = domain::CommandState::Succeeded;
    r.safety_stopped = safety_stopped;
    r.final_device_summary = device_summary();
    return r;
}

domain::SkillResult SkillBase::make_fail(const std::string& stage,
                                         const domain::Error& err) const {
    domain::SkillResult r = make_base_result();
    r.success = false;
    r.failed_stage = stage;
    r.error = err;
    r.final_state = domain::CommandState::Failed;
    r.final_device_summary = device_summary();
    if (err.category == domain::ErrorCategory::Safety) {
        r.safety_stopped = true;
    }
    return r;
}

std::string SkillBase::device_summary() const {
    if (!ctx_.store) return "";
    const auto c = ctx_.store->combined();
    std::string s = "arm=";
    s += c.arm.valid ? (c.arm.fresh ? "ok" : "stale") : "n/a";
    s += " hand=";
    s += c.hand.valid ? (c.hand.fresh ? "ok" : "stale") : "n/a";
    if (c.arm.valid && c.hand.valid) {
        s += " sync=";
        s += (c.sync_quality == domain::SyncQuality::Good    ? "good"
              : c.sync_quality == domain::SyncQuality::Skewed ? "skewed"
                                                              : "lost");
    }
    return s;
}

}  // namespace robotics::skills
