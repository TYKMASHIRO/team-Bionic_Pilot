#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "robotics/domain/results/SkillResult.hpp"

namespace robotics::domain {

/// Skill 元数据描述
struct SkillDescriptor {
    std::string id;
    std::string name;
    std::string version;
    std::string description;
    std::vector<std::string> required_resources;  ///< 需要的设备资源
    std::vector<std::string> parameters;          ///< 参数名
    std::chrono::milliseconds timeout{0};         ///< 超时（0=默认）
};

/**
 * @brief Skill 接口。所有 Skill 实现此接口。
 */
class ISkill {
public:
    virtual ~ISkill() = default;

    virtual const SkillDescriptor& descriptor() const = 0;

    /// 执行前的校验（预置条件、参数合法性）——不运动
    virtual Result validate(const std::string& parameters_json) = 0;

    /// 执行 Skill（异步实现需自行管理状态机；此处为同步阻塞执行）
    virtual SkillResult execute(const std::string& parameters_json) = 0;

    /// 请求取消
    virtual void request_cancel() = 0;
};

}  // namespace robotics::domain
