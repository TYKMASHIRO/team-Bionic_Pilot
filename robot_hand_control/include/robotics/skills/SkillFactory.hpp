#pragma once

#include <memory>
#include <string>

#include "robotics/interfaces/ISkill.hpp"
#include "robotics/skills/SkillContext.hpp"

namespace robotics::skills {

/// 依据 manifest id 构建对应 Skill 实例。
/// @return nullptr 且 error 说明（未知 id / 非法 manifest 数据）。
std::shared_ptr<domain::ISkill> make_skill(const domain::SkillDescriptor& desc,
                                           const SkillContext& ctx,
                                           std::string& error);

}  // namespace robotics::skills
