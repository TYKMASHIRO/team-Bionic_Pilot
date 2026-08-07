#include "robotics/skills/SkillRuntime.hpp"

#include <algorithm>
#include <chrono>
#include <vector>

namespace robotics::skills {

using namespace robotics::domain;

namespace {

// required_resources 字符串 → 设备类型集合
std::vector<DeviceType> to_devices(const std::string& res) {
    if (res == "arm") return {DeviceType::RobotArm};
    if (res == "hand") return {DeviceType::DexterousHand};
    if (res == "combined") {
        return {DeviceType::RobotArm, DeviceType::DexterousHand};
    }
    return {};
}

}  // namespace

SkillRuntime::SkillRuntime(std::shared_ptr<domain::IClock> clock)
    : clock_(std::move(clock)) {}

void SkillRuntime::set_resource_manager(
    std::shared_ptr<domain::ResourceManager> resources) {
    resources_ = std::move(resources);
}

SkillResult SkillRuntime::run(const std::shared_ptr<domain::ISkill>& skill,
                              const std::string& parameters_json, bool dry_run,
                              const std::function<bool()>& cancel) {
    SkillResult result;
    const SkillDescriptor& desc = skill->descriptor();
    result.skill_id = desc.id;
    result.skill_version = desc.version;
    result.config_version = "default";

    auto steady_now = [this]() {
        return clock_ ? clock_->steady_now()
                      : std::chrono::steady_clock::now();
    };
    const auto begin = steady_now();
    auto elapsed = [&begin, &steady_now]() {
        return std::chrono::duration_cast<std::chrono::milliseconds>(
            steady_now() - begin);
    };

    // ---- 资源互斥（dry-run 不运动，不占资源）----
    std::vector<DeviceType> acquired;
    auto release_all = [&]() {
        if (!resources_) return;
        for (const auto dev : acquired) resources_->release(dev, desc.id);
    };
    if (!dry_run && resources_) {
        // 展开 + 去重（防 required_resources 冗余导致重复占用同一设备）
        std::vector<DeviceType> devices;
        for (const auto& res : desc.required_resources) {
            for (const auto dev : to_devices(res)) {
                if (dev == DeviceType::Unknown) continue;
                if (std::find(devices.begin(), devices.end(), dev) ==
                    devices.end()) {
                    devices.push_back(dev);
                }
            }
        }
        for (const auto dev : devices) {
            const Result a = resources_->acquire(dev, desc.id);
            if (!a.success) {
                    result.failed_stage = "acquire_resources";
                    result.error = a.error;
                    result.final_state = CommandState::Failed;
                    result.duration_ms = elapsed();
                    release_all();
                    return result;
                }
                acquired.push_back(dev);
            }
        }

    // ---- 预置条件 + 参数校验（不运动）----
    SkillParams params;
    params.parameters_json = parameters_json;
    params.dry_run = dry_run;
    params.cancel = cancel;
    const Result v = skill->validate(params);
    if (!v.success) {
        result.failed_stage = "validate";
        result.error = v.error;
        result.final_state = CommandState::Failed;
        result.duration_ms = elapsed();
        release_all();
        return result;
    }
    if (dry_run) {
        result.success = true;
        result.failed_stage = "dry_run";
        result.final_state = CommandState::Succeeded;
        result.duration_ms = elapsed();
        release_all();
        return result;
    }

    // ---- 超时并入取消回调 ----
    SkillParams exec = params;
    const bool has_timeout = desc.timeout.count() > 0;
    if (has_timeout || cancel) {
        std::chrono::steady_clock::time_point deadline{};
        if (has_timeout) deadline = steady_now() + desc.timeout;
        std::function<bool()> base_cancel = cancel;
        exec.cancel = [this, has_timeout, deadline, base_cancel]() {
            if (has_timeout &&
                (clock_ ? clock_->steady_now() : std::chrono::steady_clock::now()) >=
                    deadline) {
                return true;
            }
            return base_cancel ? base_cancel() : false;
        };
    }

    // ---- 执行 ----
    const SkillResult r = skill->execute(exec);
    result.success = r.success;
    result.error = r.error;
    result.failed_stage = r.failed_stage;
    result.final_device_summary = r.final_device_summary;
    result.safety_stopped = r.safety_stopped;
    result.trajectory_version = r.trajectory_version;
    result.calibration_version = r.calibration_version;
    result.duration_ms = elapsed();

    if (r.success) {
        result.final_state = CommandState::Succeeded;
    } else if (r.error.category == ErrorCategory::Cancelled) {
        result.final_state = CommandState::Cancelled;
    } else if (r.error.category == ErrorCategory::Timeout) {
        result.final_state = CommandState::TimedOut;
    } else if (r.error.category == ErrorCategory::Safety) {
        result.final_state = CommandState::SafetyStopped;
        result.safety_stopped = true;
    } else {
        result.final_state = CommandState::Failed;
    }

    release_all();
    return result;
}

}  // namespace robotics::skills
