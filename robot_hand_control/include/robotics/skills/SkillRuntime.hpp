#pragma once

#include <functional>
#include <memory>
#include <string>

#include "robotics/domain/results/SkillResult.hpp"
#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/ISkill.hpp"
#include "robotics/services/ResourceManager.hpp"

namespace robotics::skills {

/**
 * @brief Skill 运行时：统一执行入口。
 *
 * 流程：资源互斥（非 dry-run）→ validate（不运动）→ [dry-run 短路] →
 * 执行（cancel/超时并入回调）→ 结果装配（final_state/时长/安全标志）。
 */
class SkillRuntime {
public:
    explicit SkillRuntime(std::shared_ptr<domain::IClock> clock = nullptr);
    ~SkillRuntime() = default;

    void set_resource_manager(
        std::shared_ptr<domain::ResourceManager> resources);

    /// 执行一个 Skill。cancel 返回 true 表示请求取消。
    domain::SkillResult run(const std::shared_ptr<domain::ISkill>& skill,
                            const std::string& parameters_json, bool dry_run,
                            const std::function<bool()>& cancel = nullptr);

private:
    std::shared_ptr<domain::IClock> clock_;
    std::shared_ptr<domain::ResourceManager> resources_;
};

}  // namespace robotics::skills
