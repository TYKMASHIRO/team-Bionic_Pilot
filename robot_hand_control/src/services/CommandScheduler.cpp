#include "robotics/services/CommandScheduler.hpp"

#include <algorithm>

#include "src/infrastructure/time/SystemClock.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"

namespace robotics::domain {

namespace {
auto& log() { return robotics::infra::Logger::instance(); }
}  // namespace

CommandScheduler::CommandScheduler(std::shared_ptr<IClock> clock)
    : clock_(clock ? std::move(clock)
                   : std::make_shared<robotics::infra::SystemClock>()) {}

CommandScheduler::~CommandScheduler() {
    stop();
}

void CommandScheduler::start() {
    if (running_.exchange(true)) return;
    worker_ = std::thread([this] { run_loop(); });
}

void CommandScheduler::stop() {
    if (!running_.exchange(false)) return;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_cancel_ = true;
    }
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

CommandId CommandScheduler::submit(Command cmd, CommandHandler handler) {
    const CommandId id = cmd.command_id.empty() ? make_command_id("sch") : cmd.command_id;
    cmd.command_id = id;
    cmd.state = CommandState::Queued;
    cmd.create_time = clock_->now();
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.emplace(std::move(cmd), std::move(handler));
    }
    cv_.notify_one();
    return id;
}

void CommandScheduler::cancel(CommandId id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (id == current_id_) {
        current_cancel_ = true;
    }
}

std::optional<CommandResult> CommandScheduler::last_result(CommandId id) const {
    std::lock_guard<std::mutex> lock(results_mutex_);
    for (auto it = results_.rbegin(); it != results_.rend(); ++it) {
        if (it->command_id == id) return *it;
    }
    return std::nullopt;
}

void CommandScheduler::run_loop() {
    while (running_) {
        std::pair<Command, CommandHandler> item;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [this] { return !running_ || !queue_.empty(); });
            if (!running_) break;
            item = std::move(queue_.front());
            queue_.pop();
        }

        Command& cmd = item.first;
        current_id_ = cmd.command_id;
        current_cancel_ = false;
        cmd.state = CommandState::Running;

        CommandResult result;
        result.command_id = cmd.command_id;
        result.type = cmd.type;
        result.final_state = CommandState::Running;

        // 执行命令（handler 返回后重新打上命令元数据，防止被覆盖）
        if (item.second) {
            result = item.second(cmd, current_cancel_);
        } else {
            result.final_state = CommandState::Failed;
            result.error = Error::make(ErrorCategory::Internal, cmd.target_device,
                                       "CommandScheduler", 1, "未注册命令处理器");
        }
        result.command_id = cmd.command_id;
        result.type = cmd.type;

        if (current_cancel_ && !result.success) {
            result.final_state = CommandState::Cancelled;
            result.error = Error::make(ErrorCategory::Cancelled, cmd.target_device,
                                       "CommandScheduler", 2, "命令已取消");
        }

        {
            std::lock_guard<std::mutex> lock(results_mutex_);
            results_.push_back(result);
            if (results_.size() > kMaxResults) {
                results_.erase(results_.begin(),
                               results_.begin() + (results_.size() - kMaxResults));
            }
        }

        log().debug("CommandScheduler",
                    "command done: " + result.to_string());
    }
}

}  // namespace robotics::domain
