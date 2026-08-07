#include "robotics/orchestration/ApplicationService.hpp"

#include "robotics/infrastructure/logging/Logger.hpp"
#include "src/infrastructure/recording/CsvRecordSink.hpp"
#include "src/trajectory/TrajectoryRepository.hpp"

namespace robotics::domain {

namespace {
auto& log() { return robotics::infra::Logger::instance(); }
}  // namespace

ApplicationService::ApplicationService(std::shared_ptr<IClock> clock,
                                       std::shared_ptr<IRobotArm> arm,
                                       std::shared_ptr<IDexterousHand> hand,
                                       std::shared_ptr<IStateStore> store,
                                       std::shared_ptr<ISafetySupervisor> safety,
                                       std::shared_ptr<StateCollector> collector,
                                       std::shared_ptr<Recorder> recorder,
                                       std::shared_ptr<ITrajectoryRepository> trajectory_repo)
    : clock_(std::move(clock)),
      arm_(std::move(arm)),
      hand_(std::move(hand)),
      store_(std::move(store)),
      safety_(std::move(safety)),
      collector_(collector
                     ? collector
                     : std::make_shared<StateCollector>(arm_, hand_, store_, clock_,
                                                        50.0, 50.0)),
      recorder_(recorder),
      trajectory_repo_(trajectory_repo ? trajectory_repo
                                       : std::make_shared<TrajectoryRepository>()) {}

Result ApplicationService::connect_all() {
    if (arm_) {
        auto r = arm_->connect();
        if (!r.success) return r;
        if (store_) store_->update_arm(arm_->get_state());
    }
    if (hand_) {
        auto r = hand_->connect();
        if (!r.success) return r;
        if (store_) store_->update_hand(hand_->get_state());
    }
    return Result::ok();
}

Result ApplicationService::disconnect_all() {
    if (hand_) hand_->disconnect();
    if (arm_) arm_->disconnect();
    return Result::ok();
}

std::vector<DeviceHealth> ApplicationService::diagnose_all() const {
    std::vector<DeviceHealth> result;
    if (arm_) result.push_back(arm_->health_check());
    if (hand_) result.push_back(hand_->health_check());
    return result;
}

CombinedRobotState ApplicationService::current_state() const {
    return store_ ? store_->combined() : CombinedRobotState{};
}

void ApplicationService::enable_real_motion() {
    if (safety_) safety_->set_real_motion_enabled(true);
    log().warn("ApplicationService", "真实运动已启用（--enable-motion）");
}

CommandId ApplicationService::submit_command(Command cmd) {
    // 一期：直接同步执行（阶段4 接入 CommandScheduler 线程）
    return cmd.command_id.empty() ? make_command_id("app") : cmd.command_id;
}

// ---- 机械臂 ----
Result ApplicationService::arm_connect() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 1, "机械臂未注册"));
    auto r = arm_->connect();
    if (r.success && store_) store_->update_arm(arm_->get_state());
    return r;
}

Result ApplicationService::arm_disconnect() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 2, "机械臂未注册"));
    return arm_->disconnect();
}

Result ApplicationService::arm_home(bool dry_run) {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 3, "机械臂未注册"));
    if (dry_run) {
        log().info("ApplicationService", "arm_home --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::RobotArm,
            "ApplicationService", 4, "真实运动未启用"));
    }
    // 安全位（一期用 Mock 语义；真实 home 在阶段2实现）
    return arm_->move_joint(ArmJointVector{0, 0, 0, 0, 0, 0, 0}, 0.2, true);
}

Result ApplicationService::arm_stop() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 5, "机械臂未注册"));
    return arm_->stop();
}

Result ApplicationService::arm_drag_teach_start(bool record) {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 6, "机械臂未注册"));
    return arm_->start_drag_teach(record);
}

Result ApplicationService::arm_drag_teach_stop() {
    if (!arm_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::RobotArm, "ApplicationService", 7, "机械臂未注册"));
    return arm_->stop_drag_teach();
}

// ---- 灵巧手 ----
Result ApplicationService::hand_connect() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 8, "灵巧手未注册"));
    auto r = hand_->connect();
    if (r.success && store_) store_->update_hand(hand_->get_state());
    return r;
}

Result ApplicationService::hand_disconnect() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 9, "灵巧手未注册"));
    return hand_->disconnect();
}

Result ApplicationService::hand_open(bool dry_run) {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 10, "灵巧手未注册"));
    if (dry_run) {
        log().info("ApplicationService", "hand_open --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::DexterousHand,
            "ApplicationService", 11, "真实运动未启用"));
    }
    return hand_->apply_preset(HandPreset::Open);
}

Result ApplicationService::hand_close(bool dry_run) {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 12, "灵巧手未注册"));
    if (dry_run) {
        log().info("ApplicationService", "hand_close --dry-run（不运动）");
        return Result::ok();
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety, DeviceType::DexterousHand,
            "ApplicationService", 13, "真实运动未启用"));
    }
    return hand_->apply_preset(HandPreset::Close);
}

Result ApplicationService::hand_stop() {
    if (!hand_) return Result::fail(Error::make(ErrorCategory::NotConnected,
        DeviceType::DexterousHand, "ApplicationService", 14, "灵巧手未注册"));
    return hand_->stop();
}

SkillResult ApplicationService::run_skill(const std::string& skill_id,
                                         const std::string& parameters_json,
                                         bool dry_run) {
    (void)parameters_json;
    (void)dry_run;
    SkillResult result;
    result.skill_id = skill_id;
    result.skill_version = "0.0.0";
    // 阶段6 实现 SkillRuntime 后接入真实执行
    result.failed_stage = "skill_runtime";
    result.error = Error::make(ErrorCategory::Unsupported, DeviceType::Combined,
        "ApplicationService", 15,
        "Skill 运行时未实现（阶段6）");
    result.success = false;
    result.final_state = CommandState::Failed;
    return result;
}

// ---- 轨迹管理（阶段5）----
Result ApplicationService::trajectory_import(const std::string& recording_dir,
                                             std::string& out_id) {
    if (!trajectory_repo_) {
        return Result::fail(Error::make(ErrorCategory::Internal,
            DeviceType::Combined, "ApplicationService", 30, "轨迹仓库未初始化"));
    }
    const Result r = trajectory_repo_->import_recording(recording_dir, out_id);
    if (r.success) {
        log().info("ApplicationService",
                   "trajectory_import dir=" + recording_dir +
                       " id=" + out_id);
    }
    return r;
}

std::vector<TrajectoryMeta> ApplicationService::trajectory_list() const {
    return trajectory_repo_ ? trajectory_repo_->list()
                            : std::vector<TrajectoryMeta>{};
}

Result ApplicationService::trajectory_load(const std::string& trajectory_id,
                                           Trajectory& out) const {
    if (!trajectory_repo_) {
        return Result::fail(Error::make(ErrorCategory::Internal,
            DeviceType::Combined, "ApplicationService", 31, "轨迹仓库未初始化"));
    }
    return trajectory_repo_->load(trajectory_id, out);
}

Result ApplicationService::trajectory_validate(
    const std::string& trajectory_id, TrajectoryValidationReport& report) const {
    Trajectory traj;
    const Result ld = trajectory_load(trajectory_id, traj);
    if (!ld.success) return ld;
    TrajectoryValidator::validate(traj, report);
    return Result::ok();
}

Result ApplicationService::trajectory_replay(
    const std::string& trajectory_id, const ReplayOptions& options,
    bool dry_run, const std::function<bool()>& cancel,
    ReplayReport& report) const {
    if (!trajectory_repo_) {
        return Result::fail(Error::make(ErrorCategory::Internal,
            DeviceType::Combined, "ApplicationService", 32, "轨迹仓库未初始化"));
    }
    Trajectory traj;
    const Result ld = trajectory_load(trajectory_id, traj);
    if (!ld.success) return ld;

    if (dry_run) {
        // 纯校验：不运动，无需设备连接
        TrajectoryValidationReport vr;
        TrajectoryValidator::validate(traj, vr);
        report = ReplayReport{};
        report.dry_run = true;
        report.success = vr.valid;
        report.total_points = traj.points.size();
        report.failed_stage = vr.valid ? "dry_run" : "validate";
        if (!vr.valid) {
            report.error = Error::make(ErrorCategory::Validation,
                DeviceType::Combined, "ApplicationService", 33, vr.to_string());
        }
        return Result::ok();
    }

    // 真实复现：需要设备 + 安全许可
    if (!arm_ || !hand_) {
        return Result::fail(Error::make(ErrorCategory::NotConnected,
            DeviceType::Combined, "ApplicationService", 34,
            "复现需要机械臂与灵巧手已注册"));
    }
    if (safety_ && !safety_->real_motion_enabled()) {
        return Result::fail(Error::make(ErrorCategory::Safety,
            DeviceType::Combined, "ApplicationService", 35,
            "真实运动未启用（需要 --enable-motion 显式许可）"));
    }
    const ReplayOptions opts = options;
    report = TrajectoryReplayer(arm_, hand_, safety_, clock_)
                 .replay(traj, opts,
                         cancel ? &cancel : nullptr);
    if (!report.success) {
        return Result::fail(report.error);
    }
    return Result::ok();
}

void ApplicationService::set_trajectory_repository(
    std::shared_ptr<ITrajectoryRepository> repo) {
    trajectory_repo_ = std::move(repo);
}

void ApplicationService::set_trajectory_data_dir(const std::string& dir) {
    if (auto repo = std::dynamic_pointer_cast<TrajectoryRepository>(
            trajectory_repo_)) {
        repo->set_data_dir(dir);
        log().info("ApplicationService", "trajectory data_dir=" + dir);
    }
}

// ---- 采集/录制（阶段4）----
Result ApplicationService::start_collection() {
    if (!collector_) return Result::ok();
    return collector_->start();
}

Result ApplicationService::stop_collection() {
    if (!collector_) return Result::ok();
    return collector_->stop();
}

Result ApplicationService::record_start(const std::string& out_dir,
                                        double rate_hz,
                                        const std::string& config_hash,
                                        const std::string& calibration_ref) {
    if (recorder_ && recorder_->is_recording()) {
        return Result::fail(Error::make(ErrorCategory::ResourceConflict,
            DeviceType::Combined, "ApplicationService", 16, "录制已在进行"));
    }
    RecordingMetadata meta;
    meta.session_id = make_session_id(clock_ ? clock_->now() : make_timestamp());
    meta.sample_rate_hz = rate_hz;
    meta.config_hash = config_hash;
    meta.calibration_ref = calibration_ref;
    meta.start_time = clock_ ? clock_->now() : make_timestamp();

    auto sink = std::make_shared<robotics::infra::CsvRecordSink>(out_dir, meta);
    if (!sink->is_open()) {
        return Result::fail(Error::make(ErrorCategory::Configuration,
            DeviceType::Combined, "ApplicationService", 17,
            "无法创建录制目录: " + out_dir));
    }
    auto recorder = std::make_shared<Recorder>(store_, clock_, sink, rate_hz);
    Result r = recorder->start();
    if (!r.success) return r;

    record_sink_ = sink;
    recorder_ = recorder;
    last_metadata_ = meta;
    log().info("ApplicationService",
               "record_start session=" + meta.session_id +
                   " rate=" + std::to_string(rate_hz));
    return r;
}

Result ApplicationService::record_stop() {
    if (!recorder_ || !recorder_->is_recording()) {
        return Result::fail(Error::make(ErrorCategory::ResourceConflict,
            DeviceType::Combined, "ApplicationService", 18, "未在录制"));
    }
    Result r = recorder_->stop();
    if (r.success) {
        last_metadata_.end_time = clock_ ? clock_->now() : make_timestamp();
        log().info("ApplicationService",
                   "record_stop session=" + last_metadata_.session_id);
    }
    return r;
}

Result ApplicationService::record_event(const std::string& event) {
    if (!recorder_ || !recorder_->is_recording()) {
        return Result::fail(Error::make(ErrorCategory::ResourceConflict,
            DeviceType::Combined, "ApplicationService", 19, "未在录制"));
    }
    recorder_->record_event(event);
    return Result::ok();
}

bool ApplicationService::recording() const {
    return recorder_ && recorder_->is_recording();
}

RecordingMetadata ApplicationService::recording_metadata() const {
    return last_metadata_;
}

}  // namespace robotics::domain
