#pragma once

#include "robotics/domain/errors/Error.hpp"

namespace robotics::realman {

/**
 * @brief RM API2 返回码 → 统一 Error 转换。
 *
 * RM API2 头文件无错误码枚举，逐函数 @return 文档约定通用返回码：
 *   0 成功
 *   1 控制器返回 false（参数错误/状态错误）
 *  -1 数据发送失败 / 未找到句柄
 *  -2 数据接收失败 / 控制器超时
 *  -3 返回值解析失败
 *  -4 因函数而异（到位校验失败 / 四代控制器不支持）
 *  -5 单线程模式超时
 *  -6 机械臂停止运动规划（外部发送停止指令）
 *  -7 三代控制器不支持
 */
domain::Error rm_to_error(int rm_rc, domain::DeviceType device,
                          const std::string& module, const std::string& op);

/// 便捷版本：rc==0 返回 Result::ok()，否则返回包含映射错误的 Result::fail()。
domain::Result rm_to_result(int rm_rc, domain::DeviceType device,
                            const std::string& module, const std::string& op);

}  // namespace robotics::realman
