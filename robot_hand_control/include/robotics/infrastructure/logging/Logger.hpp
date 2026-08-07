#pragma once

#include <mutex>
#include <string>

namespace robotics::infra {

/// 日志等级
enum class LogLevel {
    Trace = 0,
    Debug,
    Info,
    Warn,
    Error,
    Critical,
};

/**
 * @brief 简单结构化日志（一期：控制台 + 可选文件）。
 * 线程安全；日志写入不得持有设备控制锁。
 */
class Logger {
public:
    static Logger& instance();

    void configure(LogLevel level, bool console, const std::string& file_path);

    void log(LogLevel level, const std::string& module,
             const std::string& message,
             const std::string& device = "",
             const std::string& command_id = "",
             const std::string& error_code = "");

    void trace(const std::string& m, const std::string& msg) { log(LogLevel::Trace, m, msg); }
    void debug(const std::string& m, const std::string& msg) { log(LogLevel::Debug, m, msg); }
    void info(const std::string& m, const std::string& msg) { log(LogLevel::Info, m, msg); }
    void warn(const std::string& m, const std::string& msg) { log(LogLevel::Warn, m, msg); }
    void error(const std::string& m, const std::string& msg) { log(LogLevel::Error, m, msg); }

private:
    Logger() = default;
    ~Logger();
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;

    std::mutex mutex_;
    LogLevel level_ = LogLevel::Info;
    bool console_ = true;
    bool file_open_ = false;
    int file_fd_ = -1;
};

}  // namespace robotics::infra
