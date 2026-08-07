#include "robotics/skills/HandPresetSkill.hpp"

#include "robotics/skills/SkillParams.hpp"

namespace robotics::skills {

using namespace robotics::domain;

HandPresetSkill::HandPresetSkill(SkillDescriptor desc, SkillContext ctx)
    : SkillBase(std::move(desc), std::move(ctx)) {}

std::string HandPresetSkill::resolve_preset(const SkillParams& params,
                                            HandPreset& out) const {
    std::string name = param_string(params.parameters_json, "preset", "");
    if (name.empty()) name = descriptor().preset;  // manifest 固定预设（open/close）
    if (name.empty()) {
        return "缺少 preset 参数（open/close/pregrasp/custom）";
    }
    std::string err;
    if (!parse_hand_preset(name, out, err)) return err;
    return {};
}

Result HandPresetSkill::check_preconditions(const SkillParams& params) {
    Result r = require_connected_hand();
    if (!r.success) return r;

    HandPreset preset;
    const std::string err = resolve_preset(params, preset);
    if (!err.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::DexterousHand,
                                        "HandPreset", 70, err));
    }
    return Result::ok();
}

SkillResult HandPresetSkill::run(const SkillParams& params) {
    HandPreset preset;
    const std::string err = resolve_preset(params, preset);
    if (!err.empty()) {
        return make_fail("resolve_preset",
                         Error::make(ErrorCategory::Validation,
                                     DeviceType::DexterousHand,
                                     "HandPreset", 71, err));
    }
    if (cancelled(params)) {
        return make_fail("cancel",
                         Error::make(ErrorCategory::Cancelled,
                                     DeviceType::DexterousHand, "HandPreset",
                                     72, "已请求取消"));
    }
    Result r = ctx().hand->apply_preset(preset);
    if (!r.success) {
        return make_fail("apply_preset", r.error);
    }
    return make_ok();
}

}  // namespace robotics::skills
