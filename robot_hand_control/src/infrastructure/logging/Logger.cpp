#include "robotics/infrastructure/logging/Logger.hpp"

#include <cstdio>
#include <ctime>
#include <fcntl.h>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>

namespace robotics::infra {

namespace {
const char* level_name(LogLevel l) {
    switch (l) {
        case LogLevel::Trace: return "TRACE";
        case LogLevel::Debug: return "DEBUG";
        case LogLevel::Info: return "INFO";
        case LogLevel::Warn: return "WARN";
        case LogLevel::Error: return "ERROR";
        case LogLevel::Critical: return "CRITICAL";
    }
    return "?";
}

std::string timestamp_str() {
    const std::time_t t = std::time(nullptr);
    std::tm tm{};
    localtime_r(&t, &tm);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}
}  // namespace

Logger& Logger::instance() {
    static Logger logger;
    return logger;
}

Logger::~Logger() {
    if (file_fd_ >= 0) {
        ::close(file_fd_);
    }
}

void Logger::configure(LogLevel level, bool console, const std::string& file_path) {
    std::lock_guard<std::mutex> lock(mutex_);
    level_ = level;
    console_ = console;
    if (file_fd_ >= 0) {
        ::close(file_fd_);
        file_fd_ = -1;
    }
    if (!file_path.empty()) {
        file_fd_ = ::open(file_path.c_str(), O_WRONLY | O_CREAT | O_APPEND, 0644);
    }
}

void Logger::log(LogLevel level, const std::string& module,
                 const std::string& message, const std::string& device,
                 const std::string& command_id, const std::string& error_code) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (level < level_) return;

    std::ostringstream line;
    line << timestamp_str() << " [" << level_name(level) << "]"
         << " [" << module << "]";
    if (!device.empty()) line << " [dev=" << device << "]";
    if (!command_id.empty()) line << " [cmd=" << command_id << "]";
    if (!error_code.empty()) line << " [err=" << error_code << "]";
    line << " " << message << "\n";

    const std::string text = line.str();
    if (console_) {
        std::fputs(text.c_str(), stderr);
    }
    if (file_fd_ >= 0) {
        // 日志写盘失败不阻塞控制流程（忽略返回值，避免 warn_unused_result）
        const ssize_t unused = ::write(file_fd_, text.data(), text.size());
        (void)unused;
    }
}

}  // namespace robotics::infra
