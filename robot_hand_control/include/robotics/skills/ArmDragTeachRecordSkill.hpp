#pragma once

#include "robotics/skills/SkillBase.hpp"

namespace robotics::skills {

/**
 * @brief arm.drag_teach_record — 拖动示教 + 同步记录双设备。
 *
 * 参数：out_dir（录制父目录，必填）、duration_s（0=直到取消）、
 * rate_hz（采样率，默认 50）、import（默认 true，录制后导入为轨迹）。
 *
 * 流程：start_drag_teach(record) → start_recording → 等待（期间可取消）→
 * stop_recording → stop_drag_teach → import_recording。
 */
class ArmDragTeachRecordSkill : public SkillBase {
public:
    ArmDragTeachRecordSkill(domain::SkillDescriptor desc, SkillContext ctx);

protected:
    domain::Result check_preconditions(
        const domain::SkillParams& params) override;
    domain::SkillResult run(const domain::SkillParams& params) override;
};

}  // namespace robotics::skills
