#include "drivers/realman/include/RmErrorMap.hpp"

namespace robotics::realman {

domain::Error rm_to_error(int rm_rc, domain::DeviceType device,
                          const std::string& module, const std::string& op) {
    using namespace robotics::domain;
    if (rm_rc == 0) return Error::ok();

    Error e;
    e.device = device;
    e.module = module;
    e.raw_vendor_code = rm_rc;

    switch (rm_rc) {
        case 1:
            e.category = ErrorCategory::Validation;
            e.code = 1001;
            e.severity = Severity::Warning;
            e.retryable = true;
            e.message = op + ": 参数错误或机械臂状态错误";
            break;
        case -1:
            e.category = ErrorCategory::Communication;
            e.code = 1002;
            e.severity = Severity::Error;
            e.retryable = true;
            e.message = op + ": 数据发送失败/未找到句柄";
            break;
        case -2:
            e.category = ErrorCategory::Timeout;
            e.code = 1003;
            e.severity = Severity::Error;
            e.retryable = true;
            e.message = op + ": 数据接收失败/控制器超时";
            break;
        case -3:
            e.category = ErrorCategory::Protocol;
            e.code = 1004;
            e.severity = Severity::Error;
            e.message = op + ": 返回值解析失败";
            break;
        case -4:
            e.category = ErrorCategory::Motion;
            e.code = 1005;
            e.severity = Severity::Error;
            e.message = op + ": 到位设备校验失败或四代控制器不支持";
            break;
        case -5:
            e.category = ErrorCategory::Timeout;
            e.code = 1006;
            e.severity = Severity::Error;
            e.message = op + ": 单线程模式超时";
            break;
        case -6:
            e.category = ErrorCategory::Cancelled;
            e.code = 1007;
            e.severity = Severity::Info;
            e.message = op + ": 机械臂停止运动规划";
            break;
        case -7:
            e.category = ErrorCategory::Unsupported;
            e.code = 1008;
            e.severity = Severity::Error;
            e.message = op + ": 三代控制器不支持该接口";
            break;
        default:
            e.category = ErrorCategory::Internal;
            e.code = 1099;
            e.severity = Severity::Error;
            e.message = op + ": 未知 RM 返回码 " + std::to_string(rm_rc);
            break;
    }
    return e;
}

domain::Result rm_to_result(int rm_rc, domain::DeviceType device,
                            const std::string& module, const std::string& op) {
    if (rm_rc == 0) return domain::Result::ok();
    return domain::Result::fail(rm_to_error(rm_rc, device, module, op));
}

}  // namespace robotics::realman
