#pragma once

#include <string>

#include "robotics/interfaces/ISkill.hpp"

namespace robotics::skills {

/// 从 YAML 文件加载 Skill manifest 到 SkillDescriptor。
/// @return false 并给出 error 说明缺失/非法字段（id 与 required_resources 必填）。
/// 不存在的字段保持默认值；技能专属数据（safe_pose/preset 等）一并读取。
bool load_skill_manifest(const std::string& yaml_path,
                         robotics::domain::SkillDescriptor& out,
                         std::string& error);

}  // namespace robotics::skills
