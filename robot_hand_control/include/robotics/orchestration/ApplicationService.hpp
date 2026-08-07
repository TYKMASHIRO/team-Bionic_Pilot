#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/errors/Error.hpp"
#include "robotics/domain/results/SkillResult.hpp"
#include "robotics/domain/states/CombinedRobotState.hpp"
#include "robotics/domain/states/DeviceHealth.hpp"
#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/IDexterousHand.hpp"
#include "robotics/interfaces/IRecordSink.hpp"
#include "robotics/interfaces/IRobotArm.hpp"
#include "robotics/interfaces/ISafetySupervisor.hpp"
#include "robotics/interfaces/IStateStore.hpp"
#include "robotics/services/RecordingMetadata.hpp"
#include "robotics/services/Recorder.hpp"
#include "robotics/services/StateCollector.hpp"

namespace robotics::domain {

/**
 * @brief 应用服务层。
 * CLI/GUI/Agent 只调用本层；本层负责设备注册、启停、命令提交、诊断。
 */
class ApplicationService {
public:
    ApplicationService(std::shared_ptr<IClock> clock,
                       std::shared_ptr<IRobotArm> arm,
                       std::shared_ptr<IDexterousHand> hand,
                       std::shared_ptr<IStateStore> store,
                       std::shared_ptr<ISafetySupervisor> safety,
                       std::shared_ptr<StateCollector> collector = nullptr,
                       std::shared_ptr<Recorder> recorder = nullptr);

    /// 连接全部已注册设备
    Result connect_all();
    Result disconnect_all();

    /// 设备健康诊断
    std::vector<DeviceHealth> diagnose_all() const;

    /// 最新组合状态
    CombinedRobotState current_state() const;

    /// 显式启用真实运动
    void enable_real_motion();

    /// 执行单个设备命令（经 CommandScheduler）
    CommandId submit_command(Command cmd);

    // ---- 高层便捷接口（CLI 用）----
    Result arm_connect();
    Result arm_disconnect();
    Result arm_home(bool dry_run);
    Result arm_stop();
    Result arm_drag_teach_start(bool record);
    Result arm_drag_teach_stop();

    Result hand_connect();
    Result hand_disconnect();
    Result hand_open(bool dry_run);
    Result hand_close(bool dry_run);
    Result hand_stop();

    // 一期 skill 执行（占位：synchronized_replay 等阶段6实现）
    SkillResult run_skill(const std::string& skill_id,
                          const std::string& parameters_json,
                          bool dry_run);

    // ---- 采集/录制（阶段4）----
    Result start_collection();
    Result stop_collection();
    Result record_start(const std::string& out_dir, double rate_hz,
                        const std::string& config_hash,
                        const std::string& calibration_ref = "none");
    Result record_stop();
    Result record_event(const std::string& event);
    bool recording() const;
    RecordingMetadata recording_metadata() const;

private:
    std::shared_ptr<IClock> clock_;
    std::shared_ptr<IRobotArm> arm_;
    std::shared_ptr<IDexterousHand> hand_;
    std::shared_ptr<IStateStore> store_;
    std::shared_ptr<ISafetySupervisor> safety_;
    std::shared_ptr<StateCollector> collector_;
    std::shared_ptr<Recorder> recorder_;
    std::shared_ptr<IRecordSink> record_sink_;   ///< 具体 sink（CsvRecordSink）
    RecordingMetadata last_metadata_;
};

}  // namespace robotics::domain
