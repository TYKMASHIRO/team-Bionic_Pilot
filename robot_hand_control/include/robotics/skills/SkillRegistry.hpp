#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/interfaces/ISkill.hpp"

namespace robotics::skills {

/// 技能注册表：id → ISkill。线程安全。
class SkillRegistry {
public:
    /// 注册一个 Skill；重复 id 覆盖（幂等）。空 id 返回错误。
    domain::Result register_skill(std::shared_ptr<domain::ISkill> skill);

    /// 按 id 查找；未注册返回 nullptr。
    std::shared_ptr<domain::ISkill> find(const std::string& id) const;

    bool contains(const std::string& id) const;

    /// 全部注册描述（按 id 排序）。
    std::vector<domain::SkillDescriptor> descriptors() const;

    /// 全部已注册 id。
    std::vector<std::string> ids() const;

    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::map<std::string, std::shared_ptr<domain::ISkill>> skills_;
};

}  // namespace robotics::skills
