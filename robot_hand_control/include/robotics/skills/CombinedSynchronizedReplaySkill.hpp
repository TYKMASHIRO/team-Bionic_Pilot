#pragma once

#include "robotics/interfaces/ITrajectoryRepository.hpp"
#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

/**
 * @brief combined.synchronized_replay — 双设备同步复现轨迹。
 *
 * 参数：trajectory_id（必填）、speed、timeout_ms、arm_speed_ratio、align_start。
 * 前置：轨迹已导入仓库；加载 + 校验通过（dry-run 同样执行完整校验，不运动）。
 * 执行：委托 TrajectoryReplayer（校验→设备在线→起点对齐→逐点下发）。
 */
class CombinedSynchronizedReplaySkill : public SkillBase {
public:
    CombinedSynchronizedReplaySkill(domain::SkillDescriptor desc,
                                    SkillContext ctx);

protected:
    domain::Result check_preconditions(
        const domain::SkillParams& params) override;
    domain::SkillResult run(const domain::SkillParams& params) override;

private:
    domain::Trajectory cached_traj_;
};

}  // namespace robotics::skills
