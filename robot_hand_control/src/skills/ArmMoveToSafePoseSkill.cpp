#include "robotics/skills/ArmMoveToSafePoseSkill.hpp"

#include "robotics/skills/SkillParams.hpp"

namespace robotics::skills {

using namespace robotics::domain;

ArmMoveToSafePoseSkill::ArmMoveToSafePoseSkill(SkillDescriptor desc,
                                               SkillContext ctx)
    : SkillBase(std::move(desc), std::move(ctx)) {}

Result ArmMoveToSafePoseSkill::check_preconditions(const SkillParams& params) {
    Result r = require_connected_arm();
    if (!r.success) return r;

    ArmJointVector target;
    const std::string err = resolve_target(params, target);
    if (!err.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::RobotArm,
                                        "ArmMoveToSafePose", 60, err));
    }
    return Result::ok();
}

std::string ArmMoveToSafePoseSkill::resolve_target(const SkillParams& params,
                                                   ArmJointVector& out) const {
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

SkillResult ArmMoveToSafePoseSkill::run(const SkillParams& params) {
    ArmJointVector target;
    const std::string err = resolve_target(params, target);
    if (!err.empty()) {
        return make_fail("resolve_target",
                         Error::make(ErrorCategory::Validation,
                                     DeviceType::RobotArm,
                                     "ArmMoveToSafePose", 61, err));
    }
    if (cancelled(params)) {
        return make_fail("cancel",
                         Error::make(ErrorCategory::Cancelled,
                                     DeviceType::RobotArm,
                                     "ArmMoveToSafePose", 62, "已请求取消"));
    }
    Result r = ctx().arm->move_joint(target, desc().safe_pose_speed,
                                     /*block=*/true);
    if (!r.success) {
        return make_fail("move_to_safe_pose", r.error);
    }
    return make_ok();
}

}  // namespace robotics::skills
