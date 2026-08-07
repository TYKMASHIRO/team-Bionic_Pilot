#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "robotics/domain/types/DeviceType.hpp"

namespace robotics::domain {

/// 错误严重程度
enum class Severity {
    Info = 0,
    Warning,
    Error,
    Critical,
};

/// 错误分类
enum class ErrorCategory {
    None = 0,
    Configuration,    ///< 配置错误
    Communication,    ///< 通信错误（连接失败/超时/无响应）
    Protocol,         ///< 协议错误
    Device,           ///< 设备故障
    Motion,           ///< 运动错误（超限/超速/规划失败）
    Safety,           ///< 安全违规
    Cancelled,        ///< 取消
    Timeout,          ///< 超时
    Unsupported,      ///< 厂商不支持该能力
    Internal,         ///< 内部错误
    NotConnected,     ///< 未连接
    StateStale,       ///< 状态数据过期
    ResourceConflict, ///< 资源冲突
    Validation,       ///< 校验失败
};

/**
 * @brief 统一错误模型。
 * 厂商错误码只允许出现在 Error.raw_vendor_code 中，由 Adapter 转换生成。
 */
struct Error {
    ErrorCategory category = ErrorCategory::None;
    DeviceType device = DeviceType::Unknown;
    std::string module;            ///< 出错模块名（如 "RealManAdapter"）
    int code = 0;                  ///< 项目内统一错误码
    Severity severity = Severity::Error;
    bool retryable = false;        ///< 是否可重试
    std::optional<int> raw_vendor_code;  ///< 原始厂商错误码（如有）
    std::string message;           ///< 人类可读说明

    bool is_ok() const { return category == ErrorCategory::None; }
    bool is_error() const { return !is_ok(); }

    static Error ok() { return {}; }

    static Error make(ErrorCategory cat, DeviceType dev, std::string mod,
                      int code, std::string msg,
                      Severity sev = Severity::Error,
                      bool retry = false) {
        Error e;
        e.category = cat;
        e.device = dev;
        e.module = std::move(mod);
        e.code = code;
        e.severity = sev;
        e.retryable = retry;
        e.message = std::move(msg);
        return e;
    }

    std::string to_string() const;
};

/// 统一成功/失败结果（不只用 bool 表达）
struct Result {
    bool success = false;
    Error error;
    int64_t sequence = 0;  ///< 结果序号

    static Result ok(int64_t seq = 0) {
        Result r;
        r.success = true;
        r.sequence = seq;
        return r;
    }

    static Result fail(Error err, int64_t seq = 0) {
        Result r;
        r.success = false;
        r.error = std::move(err);
        r.sequence = seq;
        return r;
    }
};

std::string to_string(ErrorCategory cat);
std::string to_string(Severity sev);

}  // namespace robotics::domain
