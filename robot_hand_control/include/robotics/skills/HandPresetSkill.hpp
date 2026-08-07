#pragma once

#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

/**
 * @brief hand.open / hand.close / hand.apply_preset — 灵巧手预设。
 *
 * open/close 的预设名写死在各自 manifest 的 preset 字段；
 * apply_preset 从参数 preset 解析（open/close/pregrasp/custom）。
 */
class HandPresetSkill : public SkillBase {
public:
    HandPresetSkill(domain::SkillDescriptor desc, SkillContext ctx);

protected:
    domain::Result check_preconditions(
        const domain::SkillParams& params) override;
    domain::SkillResult run(const domain::SkillParams& params) override;

private:
    /// 解析预设：参数 preset 优先，其次 manifest preset。返回空串=成功。
    std::string resolve_preset(const domain::SkillParams& params,
                               domain::HandPreset& out) const;
};

}  // namespace robotics::skills
