// RmPassthroughModbus —— 阶段3 实现。
// 通过 RM75 高层 Modbus 寄存器 API 为 O6 SDK 提供"收发完整 Modbus RTU 帧"。
#include "drivers/linkerhand/transport/RmPassthroughModbus.hpp"

#include <cstdint>
#include <cstring>
#include <mutex>
#include <vector>

#include <rm_define.h>
#include <rm_interface.h>

#include "drivers/linkerhand/transport/ModbusFrameCodec.hpp"

namespace robotics::linkerhand {

using modbus_frame::FunctionCode;

namespace {

/// RM 多读寄存器数量范围（2<num<13 → 3..12）
constexpr int kRmMinMultiRead = 3;
constexpr int kRmMaxMultiRead = 12;
/// RM 多写寄存器数量上限（num<=10）
constexpr int kRmMaxWrite = 10;

/// 帧最小长度：slave+FC+addr2+count2+CRC2
constexpr std::size_t kMinRequestLen = 8;

}  // namespace

namespace {

/// 头文件以 void* 隐藏厂商句柄；仅在 .cpp 内转换为 rm_robot_handle*
rm_robot_handle* to_handle(void* p) { return static_cast<rm_robot_handle*>(p); }

}  // namespace

// ---------------------------------------------------------------------------
// 实现（帧解析 → RM 寄存器 API 映射 → 响应帧重组）
// ---------------------------------------------------------------------------
class RmPassthroughModbus::Impl {
public:
    Impl(void* handle, uint8_t slave_id, int timeout_ms)
        : handle_(to_handle(handle)), slave_id_(slave_id), timeout_ms_(timeout_ms) {}

    rm_robot_handle* handle() const { return handle_; }
    int timeout_ms() const { return timeout_ms_; }

    /// sendRawFrame 执行事务后缓存响应帧；receiveCompleteFrame 取出。
    void store_response(std::vector<uint8_t> frame) {
        std::lock_guard<std::mutex> lock(cache_mutex_);
        response_cache_ = std::move(frame);
    }

    int take_response(uint8_t* buf, size_t max, int timeout_ms) {
        (void)timeout_ms;  // 透传路径响应已同步缓存，无等待
        std::lock_guard<std::mutex> lock(cache_mutex_);
        if (response_cache_.empty() || buf == nullptr || max < response_cache_.size()) {
            return -1;
        }
        std::memcpy(buf, response_cache_.data(), response_cache_.size());
        const int len = static_cast<int>(response_cache_.size());
        response_cache_.clear();
        return len;
    }

    /// 读寄存器（0x04/0x03）；regs 容量 ≥ count
    int read_registers(uint8_t function, uint16_t address, size_t count,
                       int* regs) const {
        rm_peripheral_read_write_params_t params{};
        params.port = kRmPort;
        params.address = static_cast<int>(address);
        params.device = slave_id_;
        params.num = static_cast<int>(count);

        const bool input =
            (function == static_cast<uint8_t>(FunctionCode::ReadInputRegisters));
        if (count == 1) {
            return input ? rm_read_input_registers(handle_, params, regs)
                         : rm_read_holding_registers(handle_, params, regs);
        }
        return input ? rm_read_multiple_input_registers(handle_, params, regs)
                     : rm_read_multiple_holding_registers(handle_, params, regs);
    }

    /// 写单寄存器（0x06），返回 RM 返回码
    int write_single(uint16_t address, uint16_t value) const {
        rm_peripheral_read_write_params_t params{};
        params.port = kRmPort;
        params.address = static_cast<int>(address);
        params.device = slave_id_;
        params.num = 0;  // 单写无需 num
        return rm_write_single_register(handle_, params, static_cast<int>(value));
    }

    /// 写多寄存器（0x10）。RM 返回 0 即控制器接受写入。
    /// 0x10 标准响应只回显 addr+count（无需回读数据），因此不额外读回。
    /// 注：O6 只支持 0x04 读；位置目标写入后当前值会滞后，回读校验不可靠。
    bool write_multiple(uint16_t address, const uint8_t* data,
                        size_t reg_count) const {
        if (data == nullptr || reg_count == 0 || reg_count > kRmMaxWrite) {
            return false;
        }
        // RM 写多寄存器契约与读一致：data 为 2*num 个 int8（原始字节），
        // 对应 Python 封装 (c_int * (num*2))。不得打包成 16 位值。
        std::vector<int> values(reg_count * 2);
        for (size_t i = 0; i < reg_count * 2; ++i) {
            values[i] = data[i];
        }

        rm_peripheral_read_write_params_t params{};
        params.port = kRmPort;
        params.address = static_cast<int>(address);
        params.device = slave_id_;
        params.num = static_cast<int>(reg_count);

        return rm_write_registers(handle_, params, values.data()) == 0;
    }

    /// 执行一次完整事务；返回响应帧长度，失败 -1
    int execute(const uint8_t* req, size_t req_len, uint8_t* resp,
                size_t max_resp) {
        if (handle_ == nullptr || req == nullptr || resp == nullptr ||
            req_len < kMinRequestLen) {
            return -1;
        }
        // 串行化透传读/写：上层（get_state / setPosition）可能并发访问
        std::lock_guard<std::mutex> lock(op_mutex_);
        const auto info = modbus_frame::parse_request(req, req_len);
        if (!info.valid) {
            return -1;
        }
        // 从站地址不符 → 拒绝（O6 SDK 构造帧的 slave_id 应为 0x27）
        if (info.slave_id != slave_id_) {
            return -1;
        }

        switch (static_cast<FunctionCode>(info.function)) {
            case FunctionCode::ReadInputRegisters:
            case FunctionCode::ReadHoldingRegisters:
                return handle_read(info, resp, max_resp);
            case FunctionCode::WriteSingleRegister:
                return handle_write_single(info, req, resp, max_resp);
            case FunctionCode::WriteMultipleRegisters:
                return handle_write_multiple(info, req, req_len, resp, max_resp);
            default:
                return -1;  // 不支持的 FC（含异常码 0x80|FC）
        }
    }

private:
    int handle_read(const modbus_frame::RequestInfo& info, uint8_t* resp,
                    size_t max_resp) const {
        const size_t count = info.count;
        if (count == 0) {
            return -1;
        }
        if (count == 1) {
            int reg = 0;
            if (read_registers(info.function, info.address, 1, &reg) != 0) {
                return -1;
            }
            const uint16_t regs[1] = {static_cast<uint16_t>(reg & 0xFFFF)};
            return static_cast<int>(modbus_frame::build_read_response(
                resp, max_resp, info.slave_id, info.function, regs, 1));
        }
        // 多读仅支持 RM 的 3..12 个
        if (count < kRmMinMultiRead || count > kRmMaxMultiRead) {
            return -1;
        }
        // RM 多读返回"每寄存器 2 个 int8"（原始字节，对应 Python 封装 num*2）。
        // 缓冲区必须 2*count，否则 rm_read_multiple_* 越界写（heap-buffer-overflow）。
        std::vector<int> regs(count * 2);
        if (read_registers(info.function, info.address, count, regs.data()) != 0) {
            return -1;
        }
        std::vector<uint16_t> regs16(count);
        for (size_t i = 0; i < count; ++i) {
            // Modbus RTU 大端：高字节在前，低字节在后
            regs16[i] = static_cast<uint16_t>(
                ((regs[2 * i] & 0xFF) << 8) | (regs[2 * i + 1] & 0xFF));
        }
        return static_cast<int>(modbus_frame::build_read_response(
            resp, max_resp, info.slave_id, info.function, regs16.data(), count));
    }

    int handle_write_single(const modbus_frame::RequestInfo& info,
                            const uint8_t* req, uint8_t* resp,
                            size_t max_resp) const {
        const uint16_t value = static_cast<uint16_t>((req[4] << 8) | req[5]);
        if (write_single(info.address, value) != 0) {
            return -1;
        }
        return static_cast<int>(modbus_frame::build_write_single_response(
            resp, max_resp, info.slave_id, info.address, value));
    }

    int handle_write_multiple(const modbus_frame::RequestInfo& info,
                              const uint8_t* req, size_t req_len, uint8_t* resp,
                              size_t max_resp) const {
        if (req_len < 7 + static_cast<size_t>(info.byte_count)) {
            return -1;
        }
        const uint8_t* data = req + 7;  // slave+FC+addr2+count2+bytecount 后
        if (!write_multiple(info.address, data, info.count)) {
            return -1;
        }
        return static_cast<int>(modbus_frame::build_write_multiple_response(
            resp, max_resp, info.slave_id, info.address, info.count));
    }

    static constexpr int kRmPort = 1;  // 末端接口板 RS485

    rm_robot_handle* handle_;
    uint8_t slave_id_;
    int timeout_ms_;
    mutable std::mutex op_mutex_;      // 串行化透传读/写
    std::mutex cache_mutex_;
    std::vector<uint8_t> response_cache_;
};

// ---------------------------------------------------------------------------
// IModbus 接口实现
// ---------------------------------------------------------------------------
RmPassthroughModbus::RmPassthroughModbus(void* handle, uint8_t slave_id,
                                         int timeout_ms)
    : impl_(std::make_unique<Impl>(handle, slave_id, timeout_ms)) {}

RmPassthroughModbus::~RmPassthroughModbus() = default;

bool RmPassthroughModbus::isOpen() const {
    return impl_ != nullptr && impl_->handle() != nullptr;
}

void RmPassthroughModbus::close() {
    // 透传通道由 RM75 持有；关闭 RM75 连接时通道随之失效。
}

bool RmPassthroughModbus::sendRawFrame(const uint8_t* data, size_t length) {
    // O6 SDK 的 TX 回调走 sendRawFrame：发完整请求帧并立即执行 RM 透传事务，
    // 响应帧缓存到内部缓冲，供随后的 receiveCompleteFrame 取出。
    if (impl_ == nullptr) {
        return false;
    }
    std::vector<uint8_t> resp(512);
    const int n = impl_->execute(data, length, resp.data(), resp.size());
    if (n < 0) {
        return false;
    }
    resp.resize(static_cast<size_t>(n));
    impl_->store_response(std::move(resp));
    return true;
}

int RmPassthroughModbus::receiveCompleteFrame(uint8_t* buffer, size_t max_size,
                                              int timeout_ms) {
    if (impl_ == nullptr) {
        return -1;
    }
    return impl_->take_response(buffer, max_size, timeout_ms);
}

int RmPassthroughModbus::transact(const uint8_t* request, size_t request_len,
                                  uint8_t* response, size_t max_response_len,
                                  int timeout_ms) {
    // 透传单次读/写超时由 RM 侧 rm_set_modbus_mode 配置（百毫秒），
    // 本接口超时参数仅作语义保留。transact 供直接调用，不复用缓存。
    (void)timeout_ms;
    if (impl_ == nullptr) {
        return -1;
    }
    return impl_->execute(request, request_len, response, max_response_len);
}

}  // namespace robotics::linkerhand
