#include "robotics/skills/SkillFactory.hpp"

#include "robotics/skills/ArmDragTeachRecordSkill.hpp"
#include "robotics/skills/ArmMoveToSafePoseSkill.hpp"
#include "robotics/skills/CombinedSafeReleaseSkill.hpp"
#include "robotics/skills/CombinedSynchronizedReplaySkill.hpp"
#include "robotics/skills/HandPresetSkill.hpp"

namespace robotics::skills {

std::shared_ptr<domain::ISkill> make_skill(const domain::SkillDescriptor& desc,
                                           const SkillContext& ctx,
                                           std::string& error) {
    if (desc.id == "arm.move_to_safe_pose") {
        return std::make_shared<ArmMoveToSafePoseSkill>(desc, ctx);
    }
    if (desc.id == "arm.drag_teach_record") {
        return std::make_shared<ArmDragTeachRecordSkill>(desc, ctx);
    }
    if (desc.id == "hand.open" || desc.id == "hand.close" ||
        desc.id == "hand.apply_preset") {
        return std::make_shared<HandPresetSkill>(desc, ctx);
    }
    if (desc.id == "combined.synchronized_replay") {
        return std::make_shared<CombinedSynchronizedReplaySkill>(desc, ctx);
    }
    if (desc.id == "combined.safe_release") {
        return std::make_shared<CombinedSafeReleaseSkill>(desc, ctx);
    }
    error = "未知 Skill id: " + desc.id;
    return nullptr;
}

}  // namespace robotics::skills
