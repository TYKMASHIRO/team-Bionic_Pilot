#pragma once

#include "robotics/domain/states/CombinedRobotState.hpp"
#include "robotics/domain/types/Timestamp.hpp"

namespace robotics::domain {

/**
 * @brief 记录输出（写盘后台线程消费）。
 * 业务层不依赖具体文件格式（CSV/二进制/SQLite 由实现决定）。
 */
class IRecordSink {
public:
    virtual ~IRecordSink() = default;

    /// 写入一条组合状态记录
    virtual bool write_state(const CombinedRobotState& state) = 0;

    /// 写入一条事件标记（人工标记/命令/阶段）
    virtual bool write_event(Timestamp ts, const std::string& event) = 0;

    /// 写入一条命令记录
    virtual bool write_command(const std::string& command_json) = 0;

    /// 刷新到磁盘
    virtual bool flush() = 0;

    /// 丢弃帧数（队列溢出统计）
    virtual void add_dropped_frames(std::size_t count) = 0;
    virtual std::size_t dropped_frames() const = 0;
};

}  // namespace robotics::domain
