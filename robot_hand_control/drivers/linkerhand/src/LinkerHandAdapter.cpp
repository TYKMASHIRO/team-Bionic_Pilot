// LinkerHandAdapter —— 阶段3 完整实现。
// O6 经 RM75 末端 RS485 Modbus RTU 透传访问；回调注入到 RmPassthroughModbus。
#include "drivers/linkerhand/include/LinkerHandAdapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <vector>

#include <rm_define.h>
#include <rm_interface.h>

#include "api/LinkerHandApi.h"
#include "drivers/linkerhand/transport/RmPassthroughModbus.hpp"
#include "robotics/infrastructure/logging/Logger.hpp"

namespace robotics::linkerhand {

using namespace robotics::domain;

namespace {
constexpr int kO6Dof = static_cast<int>(domain::kHandDof);
constexpr int kRmEndRs485Port = 1;   // 末端接口板 RS485
constexpr int kO6Baudrate = 115200;  // O6 固定，不可改
constexpr std::size_t kRxBufMax = 256;

auto& log() { return robotics::infra::Logger::instance(); }

/// 上限位值（raw 0-255）
std::uint8_t clamp_u8(double v) {
    return static_cast<std::uint8_t>(
        std::clamp<double>(std::round(v), 0.0, 255.0));
}

std::vector<std::uint8_t> to_u8_vec(const HandJointVector& v) {
    std::vector<std::uint8_t> out;
    out.reserve(v.size());
    for (double d : v) {
        out.push_back(clamp_u8(d));
    }
    return out;
}

Error not_connected_err() {
    return Error::make(ErrorCategory::NotConnected, DeviceType::DexterousHand,
                       "LinkerHandAdapter", 101, "O6 未连接");
}
}  // namespace

// ---------------------------------------------------------------------------
// PIMPL 实现
// ---------------------------------------------------------------------------
class LinkerHandAdapter::Impl {
public:
    Impl(void* rm_handle, std::uint8_t slave_id, int timeout_ms)
        : rm_handle_(static_cast<rm_robot_handle*>(rm_handle)),
          slave_id_(slave_id),
          timeout_ms_(timeout_ms),
          modbus_(rm_handle, slave_id, timeout_ms) {}

    ~Impl() { disconnect(); }

    // ---- 连接 ----
    Result connect() {
        if (connected_) {
            return Result::ok();
        }
        if (rm_handle_ == nullptr) {
            return Result::fail(Error::make(
                ErrorCategory::Communication, DeviceType::DexterousHand,
                "LinkerHandAdapter", 102, "RM75 句柄为空，无法建立 O6 透传"));
        }

        // O6 由 RM75 末端供电（用户确认 2026-08-07）：RM75 有电则 O6 有电，
        // 因此连接顺序"先 RM75 再 O6"与供电拓扑一致；RM 断电即 O6 断电。
        // 1. 配置末端 RS485 为 Modbus RTU 主站（O6 波特率固定 115200）
        const int rc = rm_set_modbus_mode(rm_handle_, kRmEndRs485Port,
                                          kO6Baudrate, timeout_ms_ / 100);
        if (rc != 0) {
            return Result::fail(Error::make(
                ErrorCategory::Communication, DeviceType::DexterousHand,
                "LinkerHandAdapter", 103, "配置 RM75 Modbus RTU 透传失败 rc=" +
                                              std::to_string(rc)));
        }

        // 2. 构造 LinkerHandApi 并注入回调（O6 右手）
        api_ = std::make_unique<::linkerhand::api::LinkerHandApi>(
            LINKER_HAND::O6, HAND_TYPE::RIGHT, COMM_TYPE::MODBUS);
        api_->setModbusTxCallback(
            [this](std::uint8_t sid, std::uint16_t addr, const std::uint8_t* data,
                   std::uintptr_t len) -> std::int32_t {
                (void)sid;
                (void)addr;
                return modbus_.sendRawFrame(data, len) ? 0 : -1;
            });
        api_->setModbusRxCallback(
            [this](std::uint8_t sid, std::uint16_t* addr_out,
                   std::uint8_t* data_out, std::uint8_t* len_out) -> std::int32_t {
                const int len =
                    modbus_.receiveCompleteFrame(data_out, kRxBufMax, timeout_ms_);
                if (len <= 0 || data_out[0] != sid) {
                    return -1;
                }
                if (len_out) {
                    *len_out = static_cast<std::uint8_t>(len);
                }
                if (addr_out) {
                    *addr_out = 0;
                }
                return 0;
            });

        connected_ = true;
        log().info("LinkerHandAdapter", "O6 connected via RM75 passthrough");
        return Result::ok();
    }

    Result disconnect() {
        if (api_) {
            api_->freeModbusCallback();
            api_.reset();
        }
        connected_ = false;
        return Result::ok();
    }

    bool is_connected() const { return connected_; }

    // ---- 状态读取 ----
    DexterousHandState get_state() const {
        DexterousHandState st;
        if (!connected_ || !api_) {
            st.valid = false;
            return st;
        }

        const auto pos = api_->getPosition();
        if (pos.empty()) {  // SDK 读取失败返回空 vector
            st.valid = false;
            return st;
        }

        const auto spd = api_->getSpeed();
        const auto tq = api_->getTorque();
        const auto tmp = api_->getTemperature();
        const auto fc = api_->getFaultCode();

        for (int i = 0; i < kO6Dof; ++i) {
            st.position[i] = pos[i];
            if (i < static_cast<int>(spd.size())) st.velocity[i] = spd[i];
            if (i < static_cast<int>(tq.size())) st.torque[i] = tq[i];
            if (i < static_cast<int>(tmp.size())) st.temperature[i] = tmp[i];
            if (i < static_cast<int>(fc.size())) st.fault_code[i] = fc[i];
        }

        // 压力数据暂不读取（用户决策 2026-08-07）：O6 SDK 的 getForce() 请求 40 个
        // 寄存器，超过 RM75 透传多读上限（12）。透传层会同步拒绝，但 SDK 每次失败
        // 都向控制台打印"读取压力数据失败"，刷屏严重。压力列留空（shape=""/n=0），
        // 待后续阶段做"拆分为多个 ≤12 的事务"再启用。格式本就未验证（见阶段3 笔记）。
        // st.pressure = api_->getForce();

        const bool has_fault =
            std::any_of(fc.begin(), fc.end(),
                        [](std::uint8_t c) { return c != 0; });
        st.state = has_fault ? HandState::Faulted : HandState::Idle;
        st.valid = true;
        st.fresh = true;
        st.timestamp = make_timestamp();
        st.sequence = 0;  // 阶段4 由 StateStore 统一打序号
        return st;
    }

    // ---- 关节控制 ----
    Result set_joint_positions(const HandJointVector& positions) {
        auto r = require_connected();
        if (!r.success) return r;
        api_->setPosition(to_u8_vec(positions));
        return Result::ok();
    }

    Result set_joint_speeds(const HandJointVector& speeds) {
        auto r = require_connected();
        if (!r.success) return r;
        api_->setSpeed(to_u8_vec(speeds));
        return Result::ok();
    }

    Result set_torque_limits(const HandJointVector& torques) {
        auto r = require_connected();
        if (!r.success) return r;
        api_->setTorque(to_u8_vec(torques));
        return Result::ok();
    }

    // ---- 预设 ----
    Result apply_preset(HandPreset preset) {
        auto r = require_connected();
        if (!r.success) return r;
        std::array<double, kHandDof> target{};
        switch (preset) {
            case HandPreset::Open:
                target = {255, 255, 255, 255, 255, 255};
                break;
            case HandPreset::Close:
                target = {0, 0, 0, 0, 0, 0};
                break;
            case HandPreset::PreGrasp:
                target = {255, 128, 255, 255, 255, 255};
                break;
            case HandPreset::Custom:
                // 自定义由上层经 set_joint_positions 下发，本接口无厂商语义
                return Result::ok();
        }
        api_->setPosition(to_u8_vec(target));
        return Result::ok();
    }

    // ---- 停止/诊断 ----
    Result stop() {
        // O6 SDK 无可直接停止 API；保持当前位置（重发当前值即保持）
        auto r = require_connected();
        if (!r.success) return r;
        const auto cur = api_->getPosition();
        if (!cur.empty()) {
            api_->setPosition(cur);
        }
        return Result::ok();
    }

    Result clear_error() {
        // O6 的 clearFaultCode 仅 L25/L20 支持；返回 Unsupported，不伪造
        return Result::fail(Error::make(
            ErrorCategory::Unsupported, DeviceType::DexterousHand,
            "LinkerHandAdapter", 104, "O6 不支持 clearFaultCode"));
    }

    DeviceHealth health_check() {
        DeviceHealth h;
        h.device = DeviceType::DexterousHand;
        h.driver_loaded = true;
        h.sdk_version = "2.0.0";
        h.online = connected_;
        if (!connected_ || !api_) {
            h.level = HealthLevel::Fault;
            h.summary = "O6 not connected";
            return h;
        }
        const std::string ver = api_->getVersion();
        if (!ver.empty()) {
            h.firmware_version = ver;
            h.level = HealthLevel::Ok;
            h.summary = "O6 ok";
        } else {
            h.level = HealthLevel::Degraded;
            h.summary = "O6 connected, version read failed";
        }
        return h;
    }

private:
    Result require_connected() const {
        if (!connected_ || !api_) {
            return Result::fail(not_connected_err());
        }
        return Result::ok();
    }

    rm_robot_handle* rm_handle_;
    std::uint8_t slave_id_;
    int timeout_ms_;
    RmPassthroughModbus modbus_;
    std::unique_ptr<::linkerhand::api::LinkerHandApi> api_;
    bool connected_ = false;
};

// ---------------------------------------------------------------------------
// 接口转发
// ---------------------------------------------------------------------------
LinkerHandAdapter::LinkerHandAdapter(void* rm_handle, std::uint8_t slave_id,
                                     int timeout_ms)
    : impl_(std::make_unique<Impl>(rm_handle, slave_id, timeout_ms)) {}

LinkerHandAdapter::~LinkerHandAdapter() = default;

Result LinkerHandAdapter::connect() { return impl_->connect(); }
Result LinkerHandAdapter::disconnect() { return impl_->disconnect(); }
bool LinkerHandAdapter::is_connected() const { return impl_->is_connected(); }
DexterousHandState LinkerHandAdapter::get_state() const {
    return impl_->get_state();
}

Result LinkerHandAdapter::set_joint_positions(const HandJointVector& positions) {
    return impl_->set_joint_positions(positions);
}
Result LinkerHandAdapter::set_joint_speeds(const HandJointVector& speeds) {
    return impl_->set_joint_speeds(speeds);
}
Result LinkerHandAdapter::set_torque_limits(const HandJointVector& torques) {
    return impl_->set_torque_limits(torques);
}
Result LinkerHandAdapter::apply_preset(HandPreset preset) {
    return impl_->apply_preset(preset);
}
Result LinkerHandAdapter::stop() { return impl_->stop(); }
Result LinkerHandAdapter::clear_error() { return impl_->clear_error(); }
DeviceHealth LinkerHandAdapter::health_check() {
    return impl_->health_check();
}

}  // namespace robotics::linkerhand
