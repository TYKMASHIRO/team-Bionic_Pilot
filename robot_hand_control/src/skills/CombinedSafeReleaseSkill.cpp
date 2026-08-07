#include "robotics/skills/CombinedSafeReleaseSkill.hpp"

#include "robotics/skills/SkillParams.hpp"

namespace robotics::skills {

using namespace robotics::domain;

CombinedSafeReleaseSkill::CombinedSafeReleaseSkill(SkillDescriptor desc,
                                                   SkillContext ctx)
    : SkillBase(std::move(desc), std::move(ctx)) {}

std::string CombinedSafeReleaseSkill::resolve_safe_pose(
    const SkillParams& params, ArmJointVector& out) const {
    const std::vector<double> j =
        param_double_list(params.parameters_json, "joints");
    const std::vector<double>& src = !j.empty() ? j : descriptor().safe_pose;
    if (src.size() != kArmDof) {
        return "安全位需要 " + std::to_string(kArmDof) + " 个关节值，实际 " +
               std::to_string(src.size());
    }
    for (std::size_t i = 0; i < kArmDof; ++i) out[i] = src[i];
    return {};
}

Result CombinedSafeReleaseSkill::check_preconditions(
    const SkillParams& params) {
    Result r = require_connected_arm();
    if (!r.success) return r;
    r = require_connected_hand();
    if (!r.success) return r;

    ArmJointVector pose;
    const std::string err = resolve_safe_pose(params, pose);
    if (!err.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::Combined,
                                        "SafeRelease", 90, err));
    }
    return Result::ok();
}

SkillResult CombinedSafeReleaseSkill::run(const SkillParams& params) {
    ArmJointVector pose;
    const std::string err = resolve_safe_pose(params, pose);
    if (!err.empty()) {
        return make_fail("resolve_safe_pose",
                         Error::make(ErrorCategory::Validation,
                                     DeviceType::Combined, "SafeRelease", 91,
                                     err));
    }
    // 开始后即完成（finish_current_command）：阶段边界不再检查取消。
    Result r = ctx().hand->apply_preset(HandPreset::Open);
    if (!r.success) {
        return make_fail("hand_open", r.error);
    }
    r = ctx().arm->move_joint(pose, desc().safe_pose_speed, /*block=*/true);
    if (!r.success) {
        return make_fail("arm_to_safe_pose", r.error);
    }
    return make_ok();
}

}  // namespace robotics::skills
