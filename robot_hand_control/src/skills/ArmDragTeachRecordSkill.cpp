#include "robotics/skills/ArmDragTeachRecordSkill.hpp"

#include <chrono>
#include <thread>

#include "robotics/skills/SkillParams.hpp"

namespace robotics::skills {

using namespace robotics::domain;

namespace {
constexpr std::chrono::milliseconds kPollInterval{20};
}  // namespace

ArmDragTeachRecordSkill::ArmDragTeachRecordSkill(SkillDescriptor desc,
                                                 SkillContext ctx)
    : SkillBase(std::move(desc), std::move(ctx)) {}

Result ArmDragTeachRecordSkill::check_preconditions(
    const SkillParams& params) {
    Result r = require_connected_arm();
    if (!r.success) return r;

    const std::string out_dir =
        param_string(params.parameters_json, "out_dir", "");
    if (out_dir.empty()) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::RobotArm,
                                        "DragTeachRecord", 100,
                                        "缺少 out_dir 参数"));
    }
    if (param_double(params.parameters_json, "rate_hz", 50.0) <= 0) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::RobotArm,
                                        "DragTeachRecord", 101,
                                        "rate_hz 必须 > 0"));
    }
    if (param_double(params.parameters_json, "duration_s", 0.0) < 0) {
        return Result::fail(Error::make(ErrorCategory::Validation,
                                        DeviceType::RobotArm,
                                        "DragTeachRecord", 102,
                                        "duration_s 必须 >= 0"));
    }
    return Result::ok();
}

SkillResult ArmDragTeachRecordSkill::run(const SkillParams& params) {
    const std::string out_dir =
        param_string(params.parameters_json, "out_dir", "");
    const double rate_hz = param_double(params.parameters_json, "rate_hz", 50.0);
    const double duration_s =
        param_double(params.parameters_json, "duration_s", 0.0);
    const bool do_import =
        param_bool(params.parameters_json, "import", true);

    // 阶段1：开始拖动示教（RM 控制器内记录轨迹）
    Result r = ctx().arm->start_drag_teach(/*record_trajectory=*/true);
    if (!r.success) {
        return make_fail("start_drag_teach", r.error);
    }

    // 阶段2：同步记录双设备
    if (ctx().recording.start) {
        r = ctx().recording.start(out_dir, rate_hz, "", "");
        if (!r.success) {
            ctx().arm->stop_drag_teach();
            return make_fail("start_recording", r.error);
        }
    }
    if (ctx().recording.event && ctx().recording.active &&
        ctx().recording.active()) {
        ctx().recording.event("drag_teach.start");
    }

    // 阶段3：等待（时长或取消）
    bool cancelled_by_user = false;
    auto now = [this]() {
        return ctx().clock ? ctx().clock->steady_now()
                           : std::chrono::steady_clock::now();
    };
    auto deadline = duration_s > 0 ? now() + std::chrono::duration<double>(duration_s)
                                   : std::chrono::steady_clock::time_point::max();
    while (now() < deadline) {
        if (cancelled(params)) {
            cancelled_by_user = true;
            break;
        }
        std::this_thread::sleep_for(kPollInterval);
    }

    // 阶段4：受控停止（先停录制，再停示教）
    if (ctx().recording.event && ctx().recording.active &&
        ctx().recording.active()) {
        ctx().recording.event("drag_teach.stop");
    }
    if (ctx().recording.stop) {
        r = ctx().recording.stop();
        if (!r.success) {
            ctx().arm->stop_drag_teach();
            return make_fail("stop_recording", r.error);
        }
    }
    r = ctx().arm->stop_drag_teach();
    if (!r.success) {
        return make_fail("stop_drag_teach", r.error);
    }

    if (cancelled_by_user) {
        return make_fail("wait_drag",
                         Error::make(ErrorCategory::Cancelled,
                                     DeviceType::RobotArm,
                                     "DragTeachRecord", 103,
                                     "拖动示教已取消（已受控停止）"));
    }

    // 阶段5（可选）：导入为轨迹资产
    if (do_import && ctx().trajectory_repo && ctx().recording.last_session_dir) {
        const std::string session_dir = ctx().recording.last_session_dir();
        if (!session_dir.empty()) {
            std::string id;
            r = ctx().trajectory_repo->import_recording(session_dir, id);
            if (!r.success) {
                return make_fail("import_recording", r.error);
            }
            // 在设备摘要后附加导入的轨迹 id（供操作者直接复现）
            SkillResult ok = make_ok();
            ok.final_device_summary =
                ok.final_device_summary + "; traj=" + id;
            return ok;
        }
    }
    return make_ok();
}

}  // namespace robotics::skills
