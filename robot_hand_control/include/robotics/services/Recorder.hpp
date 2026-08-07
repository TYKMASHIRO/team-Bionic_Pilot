#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "robotics/domain/errors/Error.hpp"
#include "robotics/interfaces/IClock.hpp"
#include "robotics/interfaces/IRecordSink.hpp"
#include "robotics/interfaces/IStateStore.hpp"
#include "robotics/services/RecordingMetadata.hpp"

namespace robotics::domain {

/**
 * @brief 录制器：采样线程 + 有界队列 + 写盘线程。
 *
 * - 采样线程按 sample_rate_hz 读 StateStore.combined()；序号去重
 *   （arm/hand 都未变化则跳过本轮，避免采集慢于记录率时写重复行）。
 * - 有界队列：写盘线程消费；队列满丢弃新帧并计数（dropped_frames），
 *   绝不反向阻塞采样线程。
 * - record_event/record_command 非阻塞入队，经同一写盘线程落盘，
 *   保证与状态行的相对顺序一致。
 *
 * 受控停止顺序（stop）：停采样 → join → 置 drain → join 写盘 →
 * sink->add_dropped_frames → sink->flush。
 */
class Recorder {
public:
    Recorder(std::shared_ptr<IStateStore> store,
             std::shared_ptr<IClock> clock,
             std::shared_ptr<IRecordSink> sink,
             double sample_rate_hz,
             std::size_t max_queue = 4096);
    ~Recorder();  // 自动 stop()

    Recorder(const Recorder&) = delete;
    Recorder& operator=(const Recorder&) = delete;

    /// 启动写盘线程 + 采样线程
    Result start();
    /// 受控停止（见类注释）；重复调用幂等
    Result stop();
    bool is_recording() const;

    /// 非阻塞入队一条事件标记
    void record_event(const std::string& event);
    /// 非阻塞入队一条命令记录（预留）
    void record_command(const std::string& command_json);

    std::size_t dropped_frames() const;

private:
    struct Item {
        enum class Kind { State, Event, Command } kind = Kind::State;
        CombinedRobotState state;
        Timestamp ts;
        std::string text;  ///< event / command_json
    };

    void sample_loop();
    void write_loop();
    /// best-effort 节拍：sleep_until(next_deadline)；period<=0 时仅让出 CPU
    void pace(std::chrono::steady_clock::time_point& next_deadline,
              const std::chrono::nanoseconds& period);
    void enqueue(Item it);
    bool enqueue_state(const CombinedRobotState& c);

    std::shared_ptr<IStateStore> store_;
    std::shared_ptr<IClock> clock_;
    std::shared_ptr<IRecordSink> sink_;
    double sample_rate_hz_;
    std::size_t max_queue_;

    std::thread sample_thread_;
    std::thread write_thread_;
    std::atomic<bool> recording_{false};   ///< 采样进行中
    std::atomic<bool> draining_{false};    ///< 采样已停，写盘排空后退出

    std::mutex lifecycle_mutex_;           ///< 保护 start/stop 与线程句柄
    std::mutex queue_mutex_;               ///< 保护 queue_
    std::condition_variable cv_sampler_;   ///< 及时唤醒采样线程停止
    std::condition_variable cv_writer_;    ///< 唤醒写盘线程
    std::deque<Item> queue_;
    std::atomic<std::size_t> dropped_{0};

    // 去重状态（仅采样线程访问，无需加锁）：
    bool wrote_first_ = false;             ///< 已写入首帧（全缺席首帧也要落盘）
    std::uint64_t last_arm_seq_ = 0;       ///< 上次写出的 arm 序号
    std::uint64_t last_hand_seq_ = 0;      ///< 上次写出的 hand 序号
};

}  // namespace robotics::domain
