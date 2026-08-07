#pragma once

#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

/**
 * @brief combined.safe_release — 安全释放：先手张开释放抓握，再臂回安全位。
 *
 * 一旦开始即完成两个阶段（cancellation_policy=finish_current_command）：
 * 释放过程中的取消请求在阶段边界之后不再中断，保证释放完整性。
 * 安全位从 manifest safe_pose 读取，可被参数 joints 覆盖。
 */
class CombinedSafeReleaseSkill : public SkillBase {
public:
    CombinedSafeReleaseSkill(domain::SkillDescriptor desc, SkillContext ctx);

protected:
    domain::Result check_preconditions(
        const domain::SkillParams& params) override;
    domain::SkillResult run(const domain::SkillParams& params) override;

private:
    /// 解析安全位（参数优先，其次 manifest safe_pose）。返回空串=成功。
    std::string resolve_safe_pose(const domain::SkillParams& params,
                                  domain::ArmJointVector& out) const;
};

}  // namespace robotics::skills
