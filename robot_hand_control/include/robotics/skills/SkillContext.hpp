#pragma once

#include <functional>
#include <memory>
#include <string>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/IDexterousHand.hpp"
#include "robotics/interfaces/IRobotArm.hpp"
#include "robotics/interfaces/ISafetySupervisor.hpp"
#include "robotics/interfaces/IStateStore.hpp"
#include "robotics/interfaces/ITrajectoryRepository.hpp"

namespace robotics::skills {

/// 录制生命周期回调（由上层注入；Skill 不直接依赖具体 sink/文件格式）。
struct RecordingHooks {
    /// 开始录制（out_dir 输出目录；rate_hz 采样率）
    std::function<domain::Result(const std::string& out_dir, double rate_hz,
                                 const std::string& config_hash,
                                 const std::string& calibration_ref)>
        start;
    /// 停止录制
    std::function<domain::Result()> stop;
    /// 标记一条事件
    std::function<domain::Result(const std::string& event)> event;
    /// 是否正在录制
    std::function<bool()> active;
    /// 当前录制会话目录（完整路径，供导入轨迹；无会话返回空串）
    std::function<std::string()> last_session_dir;
};

/// Skill 上下文：Skill 允许使用的全部依赖（领域接口，无厂商类型）。
struct SkillContext {
    std::shared_ptr<domain::IRobotArm> arm;
    std::shared_ptr<domain::IDexterousHand> hand;
    std::shared_ptr<domain::ISafetySupervisor> safety;
    std::shared_ptr<domain::IClock> clock;
    std::shared_ptr<domain::IStateStore> store;
    std::shared_ptr<domain::ITrajectoryRepository> trajectory_repo;
    RecordingHooks recording;  ///< 录制钩子（可为空）
};

}  // namespace robotics::skills
