#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

#include "robotics/domain/commands/Command.hpp"
#include "robotics/domain/results/CommandResult.hpp"
#include "robotics/interfaces/IClock.hpp"

namespace robotics::domain {

using CommandHandler =
    std::function<CommandResult(const Command&, const std::atomic<bool>& cancel)>;

/**
 * @brief 命令调度器：串行化同一设备上的运动命令。
 * - 单线程消费队列
 * - 管理命令生命周期（Created→Queued→Running→...）
 * - 支持取消与超时
 */
class CommandScheduler {
public:
    explicit CommandScheduler(std::shared_ptr<IClock> clock = nullptr);
    ~CommandScheduler();

    CommandScheduler(const CommandScheduler&) = delete;
    CommandScheduler& operator=(const CommandScheduler&) = delete;

    /// 提交命令，返回命令 ID；handler 由调度线程调用
    CommandId submit(Command cmd, CommandHandler handler);

    /// 取消指定命令
    void cancel(CommandId id);

    /// 阻塞启动调度线程
    void start();
    /// 受控停止（清空队列，等待当前命令结束）
    void stop();

    /// 当前状态
    std::optional<CommandResult> last_result(CommandId id) const;

private:
    void run_loop();
    void transition(Command& cmd, CommandState to);

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    std::queue<std::pair<Command, CommandHandler>> queue_;
    std::thread worker_;

    std::shared_ptr<IClock> clock_;
    std::atomic<bool> running_{false};
    std::atomic<bool> current_cancel_{false};
    CommandId current_id_;

    // 命令结果缓存（最近 N 条）
    mutable std::mutex results_mutex_;
    std::vector<CommandResult> results_;
    static constexpr std::size_t kMaxResults = 64;
};

}  // namespace robotics::domain
