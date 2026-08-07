#include "robotics/services/Recorder.hpp"

#include <chrono>
#include <thread>
#include <utility>

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

Recorder::Recorder(std::shared_ptr<IStateStore> store,
                   std::shared_ptr<IClock> clock,
                   std::shared_ptr<IRecordSink> sink,
                   double sample_rate_hz,
                   std::size_t max_queue)
    : store_(std::move(store)),
      clock_(std::move(clock)),
      sink_(std::move(sink)),
      sample_rate_hz_(sample_rate_hz),
      max_queue_(max_queue) {}

Recorder::~Recorder() {
    stop();
}

Result Recorder::start() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (recording_) return Result::ok();  // 幂等
    if (!store_ || !sink_) {
        return Result::fail(Error::make(ErrorCategory::Internal, DeviceType::Unknown,
                                        "Recorder", 0, "missing store or sink",
                                        Severity::Error));
    }
    recording_ = true;
    draining_ = false;
    write_thread_ = std::thread(&Recorder::write_loop, this);
    sample_thread_ = std::thread(&Recorder::sample_loop, this);
    log().info("Recorder", "started (rate=" + std::to_string(sample_rate_hz_) +
                               " max_queue=" + std::to_string(max_queue_) + ")");
    return Result::ok();
}

Result Recorder::stop() {
    std::lock_guard<std::mutex> lock(lifecycle_mutex_);
    if (!recording_) return Result::ok();  // 幂等
    recording_ = false;
    cv_sampler_.notify_all();
    if (sample_thread_.joinable()) sample_thread_.join();
    draining_ = true;  // 排空后写盘线程退出
    cv_writer_.notify_all();
    if (write_thread_.joinable()) write_thread_.join();
    sink_->add_dropped_frames(dropped_.load());
    sink_->flush();
    log().info("Recorder", "stopped (dropped=" + std::to_string(dropped_.load()) +
                               ")");
    return Result::ok();
}

bool Recorder::is_recording() const {
    return recording_;
}

void Recorder::record_event(const std::string& event) {
    Item it;
    it.kind = Item::Kind::Event;
    it.ts = clock_ ? clock_->now() : make_timestamp();
    it.text = event;
    enqueue(std::move(it));
}

void Recorder::record_command(const std::string& command_json) {
    Item it;
    it.kind = Item::Kind::Command;
    it.text = command_json;
    enqueue(std::move(it));
}

std::size_t Recorder::dropped_frames() const {
    return dropped_.load();
}

// ---------------------------------------------------------------------------
// 采样线程
// ---------------------------------------------------------------------------
void Recorder::sample_loop() {
    auto next = std::chrono::steady_clock::now();
    const auto period = period_from_hz(sample_rate_hz_);
    while (recording_) {
        const CombinedRobotState c = store_->combined();
        // 去重：arm/hand 序号都未变化且已写过首帧则跳过，
        // 避免记录率高于采集率时写出重复行。
        if (!wrote_first_ || c.arm.sequence != last_arm_seq_ ||
            c.hand.sequence != last_hand_seq_) {
            last_arm_seq_ = c.arm.sequence;
            last_hand_seq_ = c.hand.sequence;
            wrote_first_ = true;
            enqueue_state(c);
        }
        pace(next, period);
    }
}

void Recorder::pace(std::chrono::steady_clock::time_point& next_deadline,
                    const std::chrono::nanoseconds& period) {
    if (period <= std::chrono::nanoseconds::zero()) {
        std::this_thread::yield();  // 尽速模式：仅让出 CPU
        return;
    }
    next_deadline += period;
    std::unique_lock<std::mutex> lock(queue_mutex_);
    // 已落后（单轮耗时超周期）则不补眠直接下一轮；可被 stop() 及时唤醒
    cv_sampler_.wait_until(lock, next_deadline,
                           [this] { return !recording_.load(); });
}

bool Recorder::enqueue_state(const CombinedRobotState& c) {
    Item it;
    it.kind = Item::Kind::State;
    it.state = c;
    enqueue(std::move(it));
    return true;
}

// ---------------------------------------------------------------------------
// 写盘线程
// ---------------------------------------------------------------------------
void Recorder::write_loop() {
    while (true) {
        Item it;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            cv_writer_.wait(lock, [this] {
                return !queue_.empty() || draining_.load();
            });
            if (queue_.empty() && draining_) break;  // 排空退出
            it = std::move(queue_.front());
            queue_.pop_front();
        }
        switch (it.kind) {
            case Item::Kind::State:
                sink_->write_state(it.state);
                break;
            case Item::Kind::Event:
                sink_->write_event(it.ts, it.text);
                break;
            case Item::Kind::Command:
                sink_->write_command(it.text);
                break;
        }
    }
}

// ---------------------------------------------------------------------------
// 入队（所有生产路径共用；满则丢新帧计数，绝不反压生产者）
// ---------------------------------------------------------------------------
void Recorder::enqueue(Item it) {
    bool dropped = false;
    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if (queue_.size() >= max_queue_) {
            dropped = true;
        } else {
            queue_.push_back(std::move(it));
        }
    }
    if (dropped) {
        dropped_.fetch_add(1);
    } else {
        cv_writer_.notify_one();
    }
}

}  // namespace robotics::domain
