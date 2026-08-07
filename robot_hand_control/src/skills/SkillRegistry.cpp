#include "robotics/skills/SkillRegistry.hpp"

namespace robotics::skills {

using namespace robotics::domain;

Result SkillRegistry::register_skill(std::shared_ptr<domain::ISkill> skill) {
    if (!skill || skill->descriptor().id.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::Unknown, "SkillRegistry",
                                        110, "Skill 为空或缺少 id"));
    }
    std::lock_guard<std::mutex> lock(mutex_);
    skills_[skill->descriptor().id] = std::move(skill);
    return Result::ok();
}

std::shared_ptr<domain::ISkill> SkillRegistry::find(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = skills_.find(id);
    return it == skills_.end() ? nullptr : it->second;
}

bool SkillRegistry::contains(const std::string& id) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return skills_.count(id) > 0;
}

std::vector<domain::SkillDescriptor> SkillRegistry::descriptors() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<domain::SkillDescriptor> out;
    out.reserve(skills_.size());
    for (const auto& [id, skill] : skills_) {
        out.push_back(skill->descriptor());
    }
    return out;
}

std::vector<std::string> SkillRegistry::ids() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::string> out;
    out.reserve(skills_.size());
    for (const auto& [id, skill] : skills_) {
        out.push_back(id);
    }
    return out;
}

std::size_t SkillRegistry::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return skills_.size();
}

}  // namespace robotics::skills
