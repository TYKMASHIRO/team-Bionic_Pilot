#pragma once

#include <string>

#include "robotics/interfaces/ISkill.hpp"
#include "robotics/skills/SkillContext.hpp"

namespace robotics::skills {

/**
 * @brief Skill 公共基类。
 *
 * 统一执行流程：
 *   validate()      —— 预置条件 + 参数校验（不运动）
 *   execute()       —— 预置条件 → [dry-run 短路] → 安全门（real_motion）→ run()
 * run()             —— 子类实现的状态机主体
 *
 * 子类只需实现 check_preconditions() 与 run()。
 */
class SkillBase : public domain::ISkill {
public:
    SkillBase(domain::SkillDescriptor desc, SkillContext ctx);

    const domain::SkillDescriptor& descriptor() const override { return desc_; }
    domain::Result validate(const domain::SkillParams& params) override;
    domain::SkillResult execute(const domain::SkillParams& params) override;

protected:
    /// 子类实现：预置条件/参数校验（不运动）
    virtual domain::Result check_preconditions(
        const domain::SkillParams& params) = 0;
    /// 子类实现：真实执行主体（已通过安全门与 dry-run 短路）
    virtual domain::SkillResult run(const domain::SkillParams& params) = 0;

    // ---- 辅助 ----
    /// 检查取消回调（返回 true 表示请求取消）
    bool cancelled(const domain::SkillParams& p) const {
        return p.cancel && p.cancel();
    }

    domain::Result require_arm() const;             ///< arm 已注册
    domain::Result require_hand() const;            ///< hand 已注册
    domain::Result require_connected_arm() const;   ///< arm 注册且在线
    domain::Result require_connected_hand() const;  ///< hand 注册且在线
    domain::Result require_real_motion() const;     ///< 真实运动权限

    /// 组装带 skill_id/version 的初始结果
    domain::SkillResult make_base_result() const;
    /// 成功结果（summary 自动填设备状态）
    domain::SkillResult make_ok(bool safety_stopped = false) const;
    /// 失败结果（failed_stage 为失败阶段）
    domain::SkillResult make_fail(const std::string& stage,
                                  const domain::Error& err) const;

    /// 设备状态摘要（arm/hand 是否在线 + 同步质量）
    std::string device_summary() const;

    const SkillContext& ctx() const { return ctx_; }
    domain::SkillDescriptor& desc() { return desc_; }

private:
    domain::SkillDescriptor desc_;
    SkillContext ctx_;
};

}  // namespace robotics::skills
