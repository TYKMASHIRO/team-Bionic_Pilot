#pragma once

#include <chrono>
#include <functional>
#include <string>
#include <vector>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/results/SkillResult.hpp"

namespace robotics::domain {

/// Skill 参数定义（manifest parameters 条目）
struct SkillParameter {
    std::string name;           ///< 参数名
    std::string type = "string";  ///< string | int | double | bool | preset
    bool required = false;
    std::string default_value;
    std::string description;
};

/// Skill 元数据描述（manifest 数据 + 技能专属数据）
struct SkillDescriptor {
    std::string id;
    std::string name;
    std::string version;
    std::string description;
    std::vector<std::string> required_resources;  ///< 需要的设备资源（arm/hand）
    std::vector<SkillParameter> parameters;       ///< 参数表
    std::vector<std::string> preconditions;       ///< 预置条件（人类可读；C++ 强制）
    std::vector<std::string> execution_stages;    ///< 执行阶段（状态机）
    std::vector<std::string> success_conditions;  ///< 成功判据
    std::vector<std::string> failure_conditions;  ///< 失败判据
    std::chrono::milliseconds timeout{0};         ///< 超时（0=默认/不限）
    std::string cancellation_policy;              ///< 取消策略
    std::string recovery_policy;                  ///< 恢复策略
    bool real_motion = false;                     ///< safety_profile.real_motion
    std::vector<std::string> trajectory_references;
    std::vector<std::string> calibration_references;
    std::string manifest_path;                    ///< 来源 YAML 路径

    // ---- 技能专属数据（manifest 携带，禁止硬编码在 C++）----
    std::vector<double> safe_pose;                ///< arm 安全位（rad，7 元素）
    double safe_pose_speed = 0.2;                 ///< move_joint speed_ratio
    std::string preset;                           ///< hand 预设 id（open/close/pregrasp）
};

/// Skill 执行参数
struct SkillParams {
    std::string parameters_json;    ///< 参数（JSON 字符串；一期用 yaml-cpp 解析）
    bool dry_run = false;           ///< 只校验不运动
    /// 取消回调：返回 true 表示请求取消。空=未请求。
    std::function<bool()> cancel;
};

/**
 * @brief Skill 接口。所有 Skill 实现此接口。
 *
 * validate() 只做参数语法 + 预置条件校验，**不运动**。
 * execute() 执行完整状态机；真实运动前自行检查 real_motion 权限
 * （descriptor().real_motion == true 时）。
 */
class ISkill {
public:
    virtual ~ISkill() = default;

    virtual const SkillDescriptor& descriptor() const = 0;

    /// 执行前校验（预置条件、参数合法性）——不运动
    virtual Result validate(const SkillParams& params) = 0;

    /// 执行 Skill（同步阻塞执行）
    virtual SkillResult execute(const SkillParams& params) = 0;
};

}  // namespace robotics::domain
