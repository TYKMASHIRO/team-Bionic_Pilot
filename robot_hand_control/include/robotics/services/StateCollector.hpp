#pragma once

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <thread>

#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/IDexterousHand.hpp"
#include "robotics/interfaces/IRobotArm.hpp"
#include "robotics/interfaces/IStateStore.hpp"

namespace robotics::domain {

/**
 * @brief 状态采集线程。
 * 机械臂/灵巧手各一个采集线程：get_state() → StateStore（不可变快照）。
 * 采集线程只做"获取→转换→入队"，不写文件、不执行 Skill、不等待另一设备。
 *
 * rate_hz=0 表示"尽速"（不主动 sleep，实际帧率由 get_state 耗时决定），
 * 用于真实设备（O6 经 RM75 透传 get_state 较慢）。rate_hz>0 时 best-effort
 * 节拍：单轮耗时超过周期则直接进入下一轮，不补眠、不积压。
 */
class StateCollector {
public:
    StateCollector(std::shared_ptr<IRobotArm> arm,
                   std::shared_ptr<IDexterousHand> hand,
                   std::shared_ptr<IStateStore> store,
                   std::shared_ptr<IClock> clock,
                   double arm_rate_hz = 50.0,
                   double hand_rate_hz = 50.0);
    ~StateCollector();  // 自动 stop()

    StateCollector(const StateCollector&) = delete;
    StateCollector& operator=(const StateCollector&) = delete;

    /// 启动采集线程（空设备不启线程）；重复调用幂等
    Result start();
    /// 受控停止：running=false → notify → join 双线程
    Result stop();
    bool running() const;

private:
    void arm_loop();
    void hand_loop();
    /// best-effort 节拍：sleep_until(next_deadline)；period<=0 时仅让出 CPU
    void pace(std::chrono::steady_clock::time_point& next_deadline,
              const std::chrono::nanoseconds& period);

    std::shared_ptr<IRobotArm> arm_;
    std::shared_ptr<IDexterousHand> hand_;
    std::shared_ptr<IStateStore> store_;
    std::shared_ptr<IClock> clock_;
    double arm_rate_hz_;
    double hand_rate_hz_;

    std::thread arm_thread_;
    std::thread hand_thread_;
    std::atomic<bool> running_{false};
    std::mutex mutex_;              ///< 保护 start/stop 与线程句柄
    std::condition_variable cv_;    ///< 及时唤醒停止
};

}  // namespace robotics::domain
