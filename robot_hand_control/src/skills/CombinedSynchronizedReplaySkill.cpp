#include "robotics/skills/CombinedSynchronizedReplaySkill.hpp"

#include "robotics/skills/SkillParams.hpp"
#include "robotics/trajectory/TrajectoryReplayer.hpp"
#include "robotics/trajectory/TrajectoryValidator.hpp"

namespace robotics::skills {

using namespace robotics::domain;

CombinedSynchronizedReplaySkill::CombinedSynchronizedReplaySkill(
    SkillDescriptor desc, SkillContext ctx)
    : SkillBase(std::move(desc), std::move(ctx)) {}

Result CombinedSynchronizedReplaySkill::check_preconditions(
    const SkillParams& params) {
    Result r = require_connected_arm();
    if (!r.success) return r;
    r = require_connected_hand();
    if (!r.success) return r;

    const std::string id =
        param_string(params.parameters_json, "trajectory_id", "");
    if (id.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::Combined,
                                        "SynchronizedReplay", 80,
                                        "缺少 trajectory_id 参数"));
    }
    if (!ctx().trajectory_repo) {
        return Result::fail(Error::make(ErrorCategory::Internal,
                                        DeviceType::Combined,
                                        "SynchronizedReplay", 84,
                                        "轨迹仓库未注入"));
    }
    // 加载 + 完整校验（不运动）—— dry-run 也走这里，保证校验完整。
    r = ctx().trajectory_repo->load(id, cached_traj_);
    if (!r.success) {
        return Result::fail(Error::make(
            ErrorCategory::Validation, DeviceType::Combined, "SynchronizedReplay",
            81, "轨迹加载失败: " + id + ": " + r.error.message));
    }
    TrajectoryValidationReport vr;
    TrajectoryValidator::validate(cached_traj_, vr);
    if (!vr.valid) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::Combined,
                                        "SynchronizedReplay", 82,
                                        "轨迹校验失败: " + vr.to_string()));
    }
    return Result::ok();
}

SkillResult CombinedSynchronizedReplaySkill::run(const SkillParams& params) {
    ReplayOptions opts;
    opts.speed = param_double(params.parameters_json, "speed", 1.0);
    opts.arm_speed_ratio =
        param_double(params.parameters_json, "arm_speed_ratio", 0.2);
    opts.align_start =
        param_bool(params.parameters_json, "align_start", true);
    long long t = param_int(params.parameters_json, "timeout_ms",
                            static_cast<long long>(desc().timeout.count()));
    if (t > 0) opts.timeout = std::chrono::milliseconds(t);

    if (cancelled(params)) {
        return make_fail("cancel",
                         Error::make(ErrorCategory::Cancelled,
                                     DeviceType::Combined,
                                     "SynchronizedReplay", 83, "已请求取消"));
    }

    std::function<bool()> cancel = params.cancel;
    ReplayReport report =
        TrajectoryReplayer(ctx().arm, ctx().hand, ctx().safety, ctx().clock)
            .replay(cached_traj_, opts, cancel ? &cancel : nullptr);

    SkillResult r = make_base_result();
    r.trajectory_version = cached_traj_.meta.version;
    r.final_device_summary =
        report.final_summary.empty() ? device_summary() : report.final_summary;
    r.safety_stopped = report.safety_stopped;

    if (report.success) {
        r.success = true;
        r.final_state = CommandState::Succeeded;
        return r;
    }

    r.success = false;
    r.failed_stage =
        report.failed_stage.empty() ? "replay" : report.failed_stage;
    if (report.cancelled) {
        r.error = Error::make(ErrorCategory::Cancelled, DeviceType::Combined,
                              "SynchronizedReplay", 85, "复现已取消");
        r.final_state = CommandState::Cancelled;
    } else if (report.timed_out) {
        r.error = Error::make(ErrorCategory::Timeout, DeviceType::Combined,
                              "SynchronizedReplay", 86, "复现超时");
        r.final_state = CommandState::TimedOut;
    } else {
        r.error = report.error.is_ok()
                      ? Error::make(ErrorCategory::Motion, DeviceType::Combined,
                                    "SynchronizedReplay", 87, "复现失败")
                      : report.error;
        r.final_state =
            report.safety_stopped ? CommandState::SafetyStopped
                                  : CommandState::Failed;
    }
    return r;
}

}  // namespace robotics::skills
