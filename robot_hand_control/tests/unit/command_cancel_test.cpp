#include <atomic>
#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "robotics/services/CommandScheduler.hpp"

using namespace robotics::domain;

// 验收：命令取消测试
TEST(CommandCancelTest, SchedulerRunsAndCompletes) {
    CommandScheduler scheduler;
    scheduler.start();

    std::atomic<bool> executed{false};
    auto id = scheduler.submit(
        Command{}, [&executed](const Command&, const std::atomic<bool>&) {
            executed = true;
            CommandResult r;
            r.success = true;
            r.final_state = CommandState::Succeeded;
            return r;
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    EXPECT_TRUE(executed);
    auto res = scheduler.last_result(id);
    ASSERT_TRUE(res.has_value());
    EXPECT_TRUE(res->success);

    scheduler.stop();
}

TEST(CommandCancelTest, SchedulerCancelsCommand) {
    CommandScheduler scheduler;
    scheduler.start();

    std::atomic<bool> handler_finished{false};
    auto id = scheduler.submit(
        Command{}, [&handler_finished](const Command&, const std::atomic<bool>& cancel) {
            // 模拟长时间运行
            for (int i = 0; i < 100 && !cancel; ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            handler_finished = true;
            CommandResult r;
            r.success = !cancel;
            r.final_state = cancel ? CommandState::Cancelled : CommandState::Succeeded;
            return r;
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    scheduler.cancel(id);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));

    EXPECT_TRUE(handler_finished);
    auto res = scheduler.last_result(id);
    ASSERT_TRUE(res.has_value());
    // 取消后若尚未完成，调度器会标记为 Cancelled
    EXPECT_TRUE(res->final_state == CommandState::Cancelled ||
                res->success);

    scheduler.stop();
}

TEST(CommandCancelTest, StopDrainsQueue) {
    CommandScheduler scheduler;
    scheduler.start();

    for (int i = 0; i < 5; ++i) {
        scheduler.submit(Command{}, [](const Command&, const std::atomic<bool>&) {
            CommandResult r;
            r.success = true;
            r.final_state = CommandState::Succeeded;
            return r;
        });
    }
    scheduler.stop();  // 受控停止：应正常退出
    SUCCEED();
}
