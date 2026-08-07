# session3.md — 阶段三：O6 适配层（LinkerHandAdapter + RmPassthroughModbus）

> 本文件是**任务一（阶段三）专用工作 Prompt**。下次涉及阶段3 或继续阶段4 前，先读本文件，再决定下一步动作。
> 它保存了阶段3 的**交付状态、Domain↔O6 SDK↔Modbus 三层映射、编译坑清单与架构决策**，避免重复劳动与重踩。
> 主文档：`Prompt.md`；任务一总览：`session1.md`；阶段2 执行版：`session2.md`；本文件是阶段3 的执行聚焦版。

---

## 1. 阶段3 目标

为灵巧手 LinkerHand O6 实现**厂商隔离适配层** `LinkerHandAdapter`：

1. `RmPassthroughModbus : IModbus` 传输层（★ 关键）：O6 不直连电脑，经 RM75 末端 RS485 Modbus RTU 透传。SDK 回调要求收发**完整 Modbus RTU 帧**，RM 只暴露**高层寄存器 API** —— 三层翻译。
2. `DirectSerialModbus : IModbus` 占位（接口 + 未实现，不伪造未验证 API）。
3. `LinkerHandAdapter : IDexterousHand`：包 `LinkerHandApi(O6, RIGHT, MODBUS)` + 回调注入，六通道位置/速度/转矩控制 + 状态读取 + 预设 + 错误转换。
4. 无硬件单测：帧解析 / CRC16 / 读写映射（`--wrap` 模拟 rm_* 符号）。

阶段3 只做**适配与映射**，不做状态采集/同步记录（阶段4）与 Skill（阶段6-7）。

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`dev/cyb`
- 已提交：`fc7a549`（阶段0+1）→ `ee41ccb`（阶段2）→ `4b5455d`/`f451c0e`（session1/2 文档）。阶段3 代码**已实现未提交**（待用户指示）。

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成（docs/ 文档） |
| 阶段1 工程骨架 | ✅ 完成 |
| 阶段2 RM75 Adapter | ✅ 完成 |
| 阶段3 O6 Adapter | ✅ **完成**（三 preset 构建+测试全过，未提交） |
| 阶段4-7 | ⏳ 未开始（下一步：阶段4 统一状态与同步记录） |

> ⚡ **已确认硬件事实（用户 2026-08-07）：O6 供电来自 RM75 末端**，RM75 上电即给 O6 上电，RM 断电即 O6 断电。连接顺序已与供电拓扑一致（详见 §9 与 `docs/hardware_setup.md §3`）。

**阶段3 交付物（未提交，15 文件）**：
- `drivers/linkerhand/transport/ModbusFrameCodec.{hpp,cpp}` — 纯函数 CRC16/帧解析/组帧（无厂商依赖，mock 可测）
- `drivers/linkerhand/transport/RmPassthroughModbus.{hpp,cpp}` — 三层翻译传输层（PIMPL，header 用 `void*`）
- `drivers/linkerhand/transport/DirectSerialModbus.{hpp,cpp}` — 直连串口占位（全失败，不伪造）
- `drivers/linkerhand/include/LinkerHandAdapter.hpp` / `src/LinkerHandAdapter.cpp` — O6 适配（PIMPL）
- `drivers/linkerhand/CMakeLists.txt` — 新增 `linkerhand_codec` + `linkerhand_driver` 两层
- `robot_hand_control/CMakeLists.txt` — `add_subdirectory(drivers/linkerhand)` 移到条件外（codec 始终构建）
- `tests/unit/modbus_frame_codec_test.cpp` — 11 用例
- `tests/unit/rm_passthrough_modbus_test.cpp` — 10 用例（`--wrap`）
- `tests/CMakeLists.txt` — 注册两个新测试
- `docs/vendor_api_mapping.md`、`docs/implementation_plan.md` — 阶段3 状态同步
- `docs/hardware_setup.md` — 供电事实同步；本文件 `session3.md` — 阶段3 执行聚焦版

---

## 3. 构建命令（每次开工先跑）

```bash
cd robot_hand_control
# Mock 模式（无硬件）——linkerhand_codec 始终构建，单测帧编解码
cmake --preset mock && cmake --build build/mock -j$(nproc) && ctest --test-dir build/mock
# Debug（含 realman/linkerhand 驱动，--wrap 模拟 RM 符号）
cmake --preset debug && cmake --build build/debug -j$(nproc) && ctest --test-dir build/debug
# 硬件测试（编译验证适配层能链接 RM SDK；测试本身 DISABLED 不跑）
cmake --preset hardware && cmake --build build/hardware -j$(nproc) && ctest --test-dir build/hardware
```

**阶段3 验收基线**：`mock`(8) / `debug`(10) / `hardware`(10 过 + 1 DISABLED) 全部通过。

---

## 4. Domain ↔ O6 SDK ↔ Modbus RTU 三层映射（阶段3 核心知识，直接使用）

> O6 SDK 事实详见 `session1.md §4.2`；O6 寄存器映射详见 `docs/O6_ModbusRTU_Protocol.md`。**禁造 API**，用前查 `linkerhand-cpp-sdk/include/`。

### 4.1 通信拓扑

```
上层业务（IDexterousHand，领域类型）
   │ LinkerHandAdapter（drivers/linkerhand）
   │   构造 LinkerHandApi(O6, RIGHT, MODBUS)
   │   setModbusTxCallback → sendRawFrame
   │   setModbusRxCallback → receiveCompleteFrame
   ▼
RmPassthroughModbus : IModbus
   │ ① 解析 O6 SDK 发来的完整 Modbus RTU 帧（slave/FC/addr/count/data/CRC）
   │ ② 映射到 RM75 高层寄存器 API（rm_* 符号）
   ▼
RM75 控制器（末端接口板 RS485，port=1，115200）
   │
   ▼
LinkerHand O6（从站地址 0x27 右手）
```

**三层翻译**（`RmPassthroughModbus::Impl::execute`）：
1. `ModbusFrameCodec::parse_request` 校验 CRC + 解析（slave_id/function/address/count/byte_count）。
2. 按功能码分发给 RM 寄存器 API（见 §4.2）。
3. `ModbusFrameCodec::build_*_response` 重组含 CRC 的标准响应帧缓存，供 `receiveCompleteFrame` 取走。

### 4.2 IDexterousHand ↔ O6 SDK ↔ Modbus 功能码

| IDexterousHand | O6 SDK | 底层 Modbus |
|----------------|--------|-------------|
| `connect()` | `rm_set_modbus_mode(h,1,115200,timeout/100)` 成功后构造 `LinkerHandApi` | — |
| `get_state()` | `getPosition/getSpeed/getTorque/getTemperature/getFaultCode` | 0x04 读输入寄存器（addr 0-5 位置、6-11 转矩、12-17 速度、18-23 温度、24-29 故障码）|
| `set_joint_positions()` | `setPosition(u8[6])` | 0x10 写保持寄存器 addr 0-5（byte_count=12）|
| `set_joint_speeds()` | `setSpeed(u8[6])` | 0x10 写 addr 12-17 |
| `set_torque_limits()` | `setTorque(u8[6])` | 0x10 写 addr 6-11 |
| `apply_preset(Open/Close/PreGrasp)` | `setPosition(预设)` | 0x10 写 addr 0-5 |
| `stop()` | `getPosition` 回读 → `setPosition(当前值)` | 0x04 读 + 0x10 写（保持）|
| `clear_error()` | 不支持（`clearFaultCode` 仅 L25/L20） | → Unsupported |
| `health_check()` | `getVersion()` | — |

**预设**：Open=`{255×6}`，Close=`{0×6}`，PreGrasp=`{255,128,255,255,255,255}`（O6 语义：小值弯曲、大值伸直）。

### 4.3 功能码 → RM 寄存器 API 映射（RmPassthroughModbus 内部）

| O6 功能码 | 请求 | RM API | 数量限制 |
|-----------|------|--------|----------|
| 0x04 读输入寄存器 | count=1 | `rm_read_input_registers` | 单读恰 1 |
| 0x04 读输入寄存器 | count=3..12 | `rm_read_multiple_input_registers` | 2<num<13 |
| 0x03 读保持寄存器 | 同 0x04 | `rm_read_holding_registers` / `rm_read_multiple_holding_registers` | 同 |
| 0x06 写单寄存器 | — | `rm_write_single_register` | — |
| 0x10 写多寄存器 | count≤10 | `rm_write_registers`（不回读） | num≤10 |

参数：`rm_peripheral_read_write_params_t{port=1, address, device=0x27, num}`。

**O6 SDK 回调是 Tx/Rx 分离调用**（示例 `test_o6_modbus_0.cpp` 确认）：
- Tx 回调 → `sendRawFrame`：执行 RM 透传事务，响应帧缓存到内部缓冲。
- Rx 回调 → `receiveCompleteFrame`：从缓存取出（透传路径同步完成，无等待）。
- 因此 `sendRawFrame` 内部必须**同步完成事务并缓存响应**，不能只发字节。

### 4.4 帧格式（Modbus RTU，8N1，115200）

- 请求最小长度 8 字节：`slave + FC + addr(2) + count(2) [+ byte_count + data] + CRC(2)`。
- 0x10 请求的 `byte_count` 在 `frame[6]`（= count×2）。
- CRC16：poly 0xA001，初值 0xFFFF，小端追加。测试向量 `01 03 00 00 00 0A → 0xCDC5`。
- 读响应：`slave + FC + 字节数 + 寄存器大端数据 + CRC`。
- 写响应（0x06 echo / 0x10 echo）：回显 slave + FC + addr + 值/count + CRC。

---

## 5. 阶段3 架构决策（已定，不要推翻）

1. **两层库分离**：`linkerhand_codec`（纯函数帧编解码，无厂商依赖，**始终构建**，mock 可测）+ `linkerhand_driver`（O6 适配，受 `ENABLE_LINKERHAND_DRIVER` 控制，链接 linkerhand SDK）。顶层 CMake 把 `add_subdirectory(drivers/linkerhand)` 移出条件外。
2. **PIMPL + `void*` 隔离厂商类型**：`RmPassthroughModbus` / `LinkerHandAdapter` 头文件不含任何 `rm_*`、`LinkerHandApi`；构造参数用 `void* handle`，`.cpp` 内 `static_cast<rm_robot_handle*>`。**头文件不前置声明 `rm_robot_handle`**（详见 §6.1）。
3. **Tx/Rx 分离回调**：`sendRawFrame` 同步执行 RM 事务并缓存响应，`receiveCompleteFrame` 取缓存；`transact` 是独立路径（不复用缓存），供直接调用。
4. **单事务互斥**：`op_mutex_` 串行化透传读/写（上层 get_state/setPosition 可能并发）；缓存用独立 `cache_mutex_`。
5. **0x10 写后不读回校验**：O6 只支持 0x04 读，且位置目标写入后当前值滞后 → 回读校验不可靠；RM 返回 0 即控制器接受写入，按 0x10 标准响应回显 addr+count。
6. **RM 数量限制在传输层拦截**：多读 count 非 3..12 直接返回 -1；多写 count>10 返回 false（不让错误参数漏到 RM）。
7. **get_state 失败语义**：SDK get 系列失败返回**空 vector**（非异常）→ `st.valid=false`；`clear_error` 返回 `Result::fail(Unsupported)`，不伪造。
8. **`void*` 测试句柄**：`--wrap` 测试用 `reinterpret_cast<void*>(0x1)` 作句柄，RM 函数被替换后不会真正解引用。
9. **回调线程安全**：SDK 回调可能从 SDK 内部线程调用，回调体只转发到 mutex 保护的 `RmPassthroughModbus`，不做耗时逻辑。
10. **`disconnect()` 不调 `rm_close_modbus_mode`（用户确认 2026-08-07，保持现状）**：末端 RS485 只给 O6 用；重复 connect 时 `rm_set_modbus_mode` 会重新配置。若将来末端 RS485 另作他用，需在此补关闭。
11. **急停/停止时手保持「当前位置不动」（用户确认 2026-08-07，暂时保持现状）**：`stop()` 语义是保持，不是回安全位。若阶段4 安全策略 / 阶段6 `combined.safe_release` 需要手回张开/安全位，另行实现。

---

## 6. 编译坑清单（已修复，重踩即浪费时间）

1. **`rm_robot_handle` 前置声明与 `rm_define.h` 匿名 typedef 冲突**（阶段2 记录过，阶段3 复现）
   - 症状：`conflicting declaration 'typedef struct rm_robot_handle rm_robot_handle'`。
   - 原因：`rm_define.h:954-956` 是 `typedef struct { int id; } rm_robot_handle;`，header 里再写 `struct rm_robot_handle;` 即重复定义。
   - 修复：**头文件完全不出现 `rm_robot_handle`**，公共构造函数用 `void*`，`.cpp` 内 `to_handle()`/cast 转换。对 `RmPassthroughModbus` 和 `LinkerHandAdapter` 都适用。

2. **PIMPL 指针类型不匹配**
   - 症状：`.cpp` 里 `make_unique<Impl>(...)` 但 header 声明 `Impl* impl_` → 编译错误（`std::unique_ptr` 与裸指针不兼容）。
   - 修复：header 统一 `std::unique_ptr<Impl> impl_;`，`.cpp` 用 `make_unique`。

3. **`rm_define.h` / `rm_interface.h` 找不到**
   - 症状：`fatal error: rm_define.h: 没有那个文件或目录`（linkerhand_driver 编译 RmPassthroughModbus.cpp / LinkerHandAdapter.cpp）。
   - 原因：RM API2 include 路径没给 linkerhand 目标。
   - 修复：`linkerhand_driver` 的 `target_include_directories` 加 `${RM_API2_ROOT}/include`。

4. **write_multiple 初版回读校验缺陷（设计级）**
   - 症状：0x10 写后读回校验 → 写始终失败。
   - 原因：O6 只支持 0x04 读；位置目标写入后当前值滞后，回读值与目标不符。
   - 修复：**不读回**，`rm_write_registers == 0` 即成功，按 0x10 标准响应回显 addr+count。

5. **`parse_read_response` 未对齐访问（UBSan）**
   - 症状：`reinterpret_cast<const uint16_t*>(frame+3)` 在奇数偏移触发未对齐警告。
   - 修复：`ReadResponse` 改持 `std::vector<uint16_t> values`，逐字节大端解析。

6. **测试数据错误（写多寄存器 byte_count）**
   - 症状：写入 6 个寄存器但只给 10 字节数据（实际需 12 字节），parse 失败。
   - 修复：补 `0xFF,0xFF` 两字节；新增 `make_request_all(std::vector)` 支持变长数据测试。

7. **`set_tests_properties` 找不到测试**（阶段2 坑，阶段3 避免）
   - 新增 `unit_rm_passthrough` 时先 `add_test()` 再 `set_tests_properties`（本阶段未触发，沿用阶段2 经验）。

---

## 7. 阶段3 执行回顾（验收方式）

| 项 | 结果 |
|----|------|
| mock preset 构建+测试 | ✅ 8/8 通过（含 `unit_modbus_frame_codec` 11 用例） |
| debug preset 构建+测试 | ✅ 10/10 通过（含 `unit_rm_passthrough` 10 用例，`--wrap` 模拟 rm_*） |
| hardware preset 构建+测试 | ✅ 10 过 + `hw_arm_connect` DISABLED |
| 厂商类型泄漏到上层 | ✅ header 无 rm_*/LinkerHandApi 类型（grep 验证，注释除外） |
| `linkerhand_codec` mock 可构建 | ✅（顶层 CMake 移出条件外） |
| 未验证项（需硬件） | ⚠ 见 §9 |

---

## 8. 测试覆盖说明（无硬件）

- **`unit_modbus_frame_codec`**（11 用例，纯函数）：CRC16 已知向量、CRC 校验/篡改、解析 0x04 读请求、解析 0x10 写多寄存器请求（含 byte_count）、无效 CRC 拒绝、过短帧拒绝、组读响应（含逐字节校验）、组写单/多响应、解析读响应、缓冲不足返回 0。
- **`unit_rm_passthrough`**（10 用例，`-Wl,--wrap=` 重定向 6 个 rm_* 符号）：多读 6 输入寄存器（验证响应帧+RM 参数 port=1/device=0x27）、单读 1、写多 6、写单、RM 读失败、RM 写失败、错误 slave_id 拒绝、多读 count=2 拒绝、无效 CRC 拒绝、`transact` 直接路径。
- **`--wrap` 架构**：`tests/unit/rm_passthrough_modbus_test.cpp` 提供 `extern "C" __wrap_rm_*` 定义，CMake 以 `-Wl,--wrap=rm_...` 链接选项把 linkerhand_driver 里未解析的 `rm_*` 符号替换为测试桩；测试内 `FakeRm g_fake` 记录最后一次调用的参数与写入值。`RmPassthroughModbus.cpp` 仍针对真实 RM API 头文件编译。

---

## 9. 未验证项（阶段3 无法在无硬件环境验证，阶段4 联调时实测）

> ⚡ **已确认硬件事实（用户 2026-08-07）：O6 供电来自 RM75 末端**。RM75 上电即给 O6 上电，RM75 断电/故障即 O6 断电（O6 无独立电源，执行器直接失电，无减速过程）。`LinkerHandAdapter::connect()` 的连接顺序（RM75 → 透传 → O6）已与供电拓扑一致。详见 `docs/hardware_setup.md §3`。

| 项 | 说明 |
|----|------|
| RM75 透传时序/波特率 | `rm_set_modbus_mode(h,1,115200,timeout)` 实机是否稳定收发 O6 帧；timeout 当前默认 500ms→RM 侧 5 单位，需实测调优 |
| O6 实际响应帧 | `parse_read_response` 假设标准 Modbus 04 响应（slave/FC/bytecount/data/CRC），实机需比对 SDK 期望帧 |
| 压力数据格式 | 华威科 10×4 点阵，寄存器 45-87；**用户确认 2026-08-07：暂时不需要**，`getForce()` 保持原样放入 `st.pressure`，不投入格式验证；若阶段7 需握力判断（判断抓稳操纵杆）再启动 |
| O6 固件启动时间 | 由 RM75 末端供电；RM75 冷启动后 O6 启动时间待测，首帧读写可能需重试 |
| `rm_write_registers` 行为 | 是否要求严格 8N1 帧间隔、O6 是否丢弃过快连续帧 |
| 单通道控制 | 无单通道 set 方法，需切片 vector；行为依赖实测 |

---

## 10. 下一步：阶段4（统一状态与同步记录）

阶段3 已让 `IDexterousHand` 有真实实现。阶段4 把双设备状态统一进 `StateStore`：

| 子任务 | 要点 |
|--------|------|
| StateStore | 不可变快照，线程安全；`CombinedRobotState` = 臂状态 + 手状态 + 时间差 + 同步质量 |
| IClock | steady + system 双时间戳 |
| Recorder | 有界队列 + 后台写盘线程 + 丢帧统计 + 事件标记 + 录制元数据 |
| 阶段4 验收 | Mock 双设备连续记录；单设备离线不崩溃；文件可读；时间戳单调 |

**阶段4 入口**：`LinkerHandAdapter::get_state()` 已返回 `DexterousHandState`（`valid/fresh/timestamp/sequence=0`，sequence 阶段4 由 StateStore 统一打序号），可直接接入；`RealManAdapter::get_state()`（阶段2）同构。

**⚠ 阶段7 前置需求（用户确认 2026-08-07）**：座舱动作有**单手指动作**（`press_button` 单指按按钮、`rotate_knob` 拧旋钮）。O6 SDK 无单通道 set 方法，阶段7 前需实现单通道控制辅助：**读全通道 → 改单指 → 全写回**（`read_current_positions()` 已有，需补一个「改单指后全量写回」的辅助）。阶段4/5 不涉及。

---

## 11. 硬性约束（违反即返工）

- 不得编造厂商 API；使用前先查本地头文件（`RM_API2/C++/include/`、`linkerhand-cpp-sdk/include/`）。
- 每个阶段结束必须可编译、可测试。
- 真实运动默认禁止；未配置的安全项默认拒绝运动。
- 厂商类型/错误码/句柄不得泄漏到上层业务模块（头文件 `void*` + PIMPL）。
- 不允许在厂商 SDK 回调线程中执行耗时业务逻辑。
- 资料缺失时建 TODO + Unsupported 返回值，不得伪造实现伪装成功（`DirectSerialModbus` 全失败即此原则）。

---

## 12. 相关文档索引

| 文档 | 用途 |
|------|------|
| `Prompt.md` | 完整工程规范（总纲） |
| `session1.md` | 任务一总览：阶段0-1 状态、SDK 审计事实（RM §4.1 / O6 §4.2 / Modbus 透传 §4.3） |
| `session2.md` | 阶段2 RM75 适配层执行版（含 `rm_robot_handle` 坑 §6.1） |
| `robot_hand_control/docs/vendor_api_mapping.md` | 厂商 API 逐项映射（阶段3 已同步） |
| `robot_hand_control/docs/implementation_plan.md` | 阶段0-7 计划（阶段3 标 ✅） |
| `robot_hand_control/docs/architecture.md` | 分层架构 |
| `robot_hand_control/docs/threading_model.md` | 线程模型 |
| `robot_hand_control/docs/O6_ModbusRTU_Protocol.md` | O6 寄存器完整映射（阶段3 必读） |
