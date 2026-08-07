#include "robotics/services/StateCollector.hpp"

#include <chrono>
#include <thread>

#include "robotics/infrastructure/logging/Logger.hpp"

namespace robotics::domain {

namespace {
auto& log() { return robotics::infra::Logger::instance(); }

std::chrono::nanoseconds period_from_hz(double hz) {
    if (hz <= 0.0) return std::chrono::nanoseconds::zero();
    const double ns = 1e9 / hz;
    return std::chrono::nanoseconds(static_cast<std::int64_t>(ns));
}
}  // namespace

StateCollector::StateCollector(std::shared_ptr<IRobotArm> arm,
                               std::shared_ptr<IDexterousHand> hand,
                               std::shared_ptr<IStateStore> store,
                               std::shared_ptr<IClock> clock,
                               double arm_rate_hz, double hand_rate_hz)
    : arm_(std::move(arm)),
      hand_(std::move(hand)),
      store_(std::move(store)),
      clock_(std::move(clock)),
      arm_rate_hz_(arm_rate_hz),
      hand_rate_hz_(hand_rate_hz) {}

StateCollector::~StateCollector() {
    stop();
}

Result StateCollector::start() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (running_) return Result::ok();

    running_ = true;
    if (arm_ && store_) {
        arm_thread_ = std::thread(&StateCollector::arm_loop, this);
    }
    if (hand_ && store_) {
        hand_thread_ = std::thread(&StateCollector::hand_loop, this);
    }
    log().info("StateCollector", "started (arm_rate=" + std::to_string(arm_rate_hz_) +
                                     " hand_rate=" + std::to_string(hand_rate_hz_) + ")");
    return Result::ok();
}

Result StateCollector::stop() {
    std::thread arm_t, hand_t;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!running_) return Result::ok();
        running_ = false;
        cv_.notify_all();
        arm_t = std::move(arm_thread_);
        hand_t = std::move(hand_thread_);
    }
    if (arm_t.joinable()) arm_t.join();
    if (hand_t.joinable()) hand_t.join();
    log().info("StateCollector", "stopped");
    return Result::ok();
}

bool StateCollector::running() const {
    return running_;
}

void StateCollector::arm_loop() {
    auto next = std::chrono::steady_clock::now();
    const auto period = period_from_hz(arm_rate_hz_);
    while (running_) {
        if (arm_ && store_) {
            store_->update_arm(arm_->get_state());
        }
        pace(next, period);
    }
}

void StateCollector::hand_loop() {
    auto next = std::chrono::steady_clock::now();
    const auto period = period_from_hz(hand_rate_hz_);
    while (running_) {
        if (hand_ && store_) {
            store_->update_hand(hand_->get_state());
        }
        pace(next, period);
    }
}

void StateCollector::pace(std::chrono::steady_clock::time_point& next_deadline,
                          const std::chrono::nanoseconds& period) {
    if (period <= std::chrono::nanoseconds::zero()) {
        // 尽速模式：仅让出 CPU，避免忙等吃满核
        std::this_thread::yield();
        return;
    }
    next_deadline += period;
    std::unique_lock<std::mutex> lock(mutex_);
    // 已落后（单轮耗时超周期）则不补眠直接下一轮
    cv_.wait_until(lock, next_deadline,
                   [this] { return !running_.load(); });
}

}  // namespace robotics::domain
