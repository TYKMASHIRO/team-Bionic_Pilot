#pragma once

#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

/**
 * @brief arm.move_to_safe_pose — 机械臂回到安全位。
 *
 * 目标位从 manifest 的 safe_pose（rad，7 元素）读取，可被参数
 * joints（7 元素数组）覆盖。真实运动需要权限。
 */
class ArmMoveToSafePoseSkill : public SkillBase {
public:
    ArmMoveToSafePoseSkill(domain::SkillDescriptor desc, SkillContext ctx);

protected:
    domain::Result check_preconditions(
        const domain::SkillParams& params) override;
    domain::SkillResult run(const domain::SkillParams& params) override;

private:
    /// 解析目标关节（参数优先，其次 manifest safe_pose）。返回空串=成功。
    std::string resolve_target(const domain::SkillParams& params,
                               domain::ArmJointVector& out) const;
};

}  // namespace robotics::skills
