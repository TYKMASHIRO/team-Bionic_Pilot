# session4.md — 阶段四：统一状态与同步记录

> 本文件是**任务一（阶段四）专用工作 Prompt**。下次涉及阶段4 或继续阶段5 前，先读本文件，再决定下一步动作。
> 它保存了阶段4 的**交付状态、真实硬件验证结论（含 RM 多寄存器契约修复）、真机运动链路验证、编译坑清单与架构决策**，避免重复劳动与重踩。
> 主文档：`Prompt.md`；任务一总览：`session1.md`；阶段2 执行版：`session2.md`；阶段3 执行版：`session3.md`；本文件是阶段4 的执行聚焦版。

---

## 1. 阶段4 目标

把 RM75 机械臂 + LinkerHand O6 灵巧手**双设备状态统一进 StateStore**，实现**连续采集 + 同步录制**：

1. **StateStore 统一打序号**：设备适配器一律 `sequence=0`，序号单一来源在 StateStore；`SyncQuality::Lost`；「RM75 断连视同 O6 断连」离线语义（O6 由 RM 末端供电）。
2. **StateCollector**：arm/hand 各一个采集线程 → `get_state()` → StateStore（不可变快照）。
3. **Recorder**：有界队列 + 后台写盘线程 + 丢帧统计 + 事件标记 + 录制元数据。
4. **CsvRecordSink**：states.csv（固定 110 列）/events.csv/commands.csv/metadata.txt。
5. **真实硬件接入**：`RealManAdapter::native_handle()`（`void*`）+ CLI `--real`，连接 arm→hand、断开 hand→arm。

阶段4 只做**状态统一与录制**，不做轨迹回放（阶段5）与 Skill（阶段6-7）。

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`dev/cyb`
- 已提交：`fc7a549`（阶段0+1）→ `ee41ccb`（阶段2）→ `4b5455d`/`f451c0e`（session1/2 文档）→ `1a62130`（阶段3）→ `7051c4a`（session3 文档）。阶段4 代码**本次提交**。

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成（docs/ 文档） |
| 阶段1 工程骨架 | ✅ 完成 |
| 阶段2 RM75 Adapter | ✅ 完成 |
| 阶段3 O6 Adapter | ✅ 完成 |
| 阶段4 统一状态与同步记录 | ✅ **完成**（代码 + Mock 测试 + 真实硬件验证，本次提交） |
| 阶段5-7 | ⏳ 未开始（下一步：阶段5 轨迹复现） |

> ⚡ **已确认硬件事实（用户 2026-08-07）：O6 供电来自 RM75 末端**，RM75 上电即给 O6 上电，RM 断电即 O6 断电。连接顺序（arm→hand）已与供电拓扑一致。

**阶段4 交付物（本提交，18 modified + 11 new 文件）**：
- `src/services/StateStore.{hpp,cpp}` — `arm_seq_`/`hand_seq_` 统一打号；`combined()`：hand 依赖 arm、`SyncQuality::Good/Skewed/Lost/Unknown`（50ms 阈值）
- `include/robotics/services/RecordingMetadata.hpp` — 纯头：meta 结构体 + `make_session_id`（文件名安全）
- `include/robotics/services/StateCollector.{hpp,cpp}`（新建）— arm/hand 双采集线程，rate=0 尽速
- `src/infrastructure/recording/CsvRecordSink.{hpp,cpp}`（新建）— 110 列落盘，`*_valid` 判定在场，metadata 汇总块幂等
- `include/robotics/services/Recorder.{hpp,cpp}`（新建）— 采样+去重+有界队列+丢帧计数+受控停止
- `include/robotics/orchestration/ApplicationService.hpp` / `src/orchestration/ApplicationService.cpp` — 构造加默认参数 `collector/recorder`；`start/stop_collection`、`record_start/stop/event`、`recording()`、`recording_metadata()`
- `drivers/realman/include/RealManAdapter.hpp` / `src/RealManAdapter.cpp` — `native_handle()`（`void*`，加锁读句柄）
- `apps/robotctl/main.cpp` — `record <out_dir> --duration <s> [--rate <hz>]` 前台循环 + Ctrl-C；`--real`；HAVE_*_DRIVER 宏条件编译
- `apps/CMakeLists.txt` — 注入 `HAVE_REALMAN_DRIVER`/`HAVE_LINKERHAND_DRIVER` 宏
- Mock 归零：`MockRobotArm/MockDexterousHand` 的 `++sequence_` → `sequence=0`；`state_snapshot_test` 断言 1→0
- 测试 5 个新建：`unit_state_store_seq` / `unit_state_collector` / `unit_recorder` / `unit_csv_sink` / `integration_mock_record`
- 顶层 CMake：robotics_core 源列表追加 StateCollector/Recorder/CsvRecordSink
- 本文件 `session4.md`

---

## 3. 构建命令（每次开工先跑）

```bash
cd robot_hand_control
# Mock 模式（无硬件，两驱动 OFF）
cmake --preset mock && cmake --build build/mock -j$(nproc) && ctest --test-dir build/mock
# Debug（含 realman/linkerhand 驱动；真实设备用此构建）
cmake --preset debug && cmake --build build/debug -j$(nproc) && ctest --test-dir build/debug
# ASan（地址/UB 检测；真实调试内存问题必用）
cmake --preset asan && cmake --build build/asan -j$(nproc) && ctest --test-dir build/asan
# 硬件测试（编译验证；测试本身 DISABLED）
cmake --preset hardware && cmake --build build/hardware -j$(nproc) && ctest --test-dir build/hardware
```

**阶段4 验收基线**：`mock`(13) / `debug`(15) / `asan`(15) 全部通过。

**真实设备 CLI 用法**：
```bash
# 非运动：连续采集 + 同步录制（设备已装好，每步真实动作须用户确认）
./build/debug/apps/robotctl --real record data/recordings --duration 10 --rate 5
# 真实运动（须 --enable-motion）
./build/debug/apps/robotctl --real --enable-motion hand preset open
./build/debug/apps/robotctl --real --enable-motion hand preset close
```

---

## 4. 阶段4 架构决策（已定，不要推翻）

1. **序号单一来源**：设备适配器（RM/O6/Mock）`get_state()` 一律 `sequence=0`；StateStore 在 `update_arm/update_hand` 时统一打号（`arm_seq_`/`hand_seq_`）。`latest_*()` 也携带统一序号。
2. **hand 依赖 arm**：`combined()` 中 `hand_ok = hand_ && hand->valid && arm_ok`——即使残留有效 hand 快照，RM 断连也视同 O6 断连（不产生「arm 无、hand 有」的物理不可能行）。
3. **sync 语义**：双在场时差≤50ms → `Good`，>50ms → `Skewed`；仅 arm 在场 → `Lost`；都不在 → `Unknown`。`c.timestamp` 取 arm。
4. **受控停止顺序（红线）**：`record_stop`（Recorder 停采样→join→排空→join 写盘→`add_dropped_frames`→`flush`）→ `stop_collection`（join 采集）→ `disconnect_all`（hand→arm）。
5. **写盘不阻塞采集**：Recorder 有界队列满丢新帧计数，绝不反压采样线程；`record_event/record_command` 走同一写盘线程，保证与状态行相对顺序一致。
6. **序号去重**：Recorder 采样只写首帧 + arm/hand 序号任一变化的帧（记录率高于采集率时不写重复行）。
7. **厂商类型不泄漏**：`native_handle()` 返回 `void*`（RM 句柄上抛，所有权仍归 RealManAdapter）；上层头不 include 厂商头。
8. **真实路径 wiring**：RM75 `connect()` → `native_handle()` → `LinkerHandAdapter(h,...)` → `connect()`；`StateCollector` rate=0 尽速（帧率由 get_state 实际耗时决定）；`record --rate 5`（O6 get_state ≈6 次 Modbus 透传，组合帧率数 Hz）。
9. **CLI 全局开关解析**：`--real`/`--enable-motion`/`--duration`/`--rate` 过滤后取命令 token（避免开关打头破坏 `args[0]` 逻辑）。
10. **CSV 缺席编码**：设备 `valid=false` 时数值列写 0、字符串列写空串、`*_valid=0`；消费方以 `*_valid` 判定在场。

---

## 5. ★ 真实硬件调试发现（阶段4 最有价值结论，阶段5/6 必读）

### 5.1 RM 多寄存器读/写契约：`data` 是 `2*num` 个 int8 原始字节，不是 num 个 int

**现象**：真实录制启动即 `malloc(): unaligned fastbin chunk detected` 崩溃。ASan 精确定位：
```
WRITE of size 48（rm_read_multiple_input_registers memcpy）
位于 24 字节区域右侧 0 字节（regs 只分配了 count=6 个 int）
```

**根因**：RM API2 的 `rm_read_multiple_input_registers` / `rm_read_multiple_holding_registers` / `rm_write_registers` 的 `data` 参数是 **2×num 个 int8（每个寄存器 2 个原始字节）**，不是 num 个 int。铁证：官方 Python 封装 `data_num = int(read_params.num * 2)`；官方日志 `num:5 → data:[10 个元素]`；文档标注「数据类型 int8」。

**修复**（`RmPassthroughModbus.cpp`）：
- 多读：缓冲区 `2*count`；按 Modbus RTU 大端重组 `regs16[i] = (regs[2i]&0xFF)<<8 | (regs[2i+1]&0xFF)`。
- 多写：逐字节拷贝 `2*reg_count`（原实现打包成 16 位同样违反契约，对 0-255 位置值碰巧因高字节为 0 而"看似可用"，已一并修复）。
- **单读** `rm_read_input_registers`/`rm_read_holding_registers` 返回 1 个 int16（无此问题）。
- 测试 mock 已同步为写 `2*params.num` 并断言 `written_values.size()==12`，忠实模拟真实 RM，防回归（若代码再少分配会立即 ASan 崩）。

> **这是本阶段最重要的坑。** 若阶段5/6 接触任何 RM 多寄存器读写，直接沿用此契约。

### 5.2 O6 压力读取被跳过（用户决策 2026-08-07）

- O6 SDK `getForce()` 请求 **40 个寄存器**（响应 87 字节），超过 RM75 透传多读上限（3..12）。
- 透传层同步拒绝（不阻塞、不崩溃），但 **SDK 每次失败向控制台硬编码打印「读取压力数据失败」，无法静音**，刷屏严重。
- **已按用户决策跳过**：`LinkerHandAdapter::get_state()` 注释掉 `api_->getForce()`。压力列留空（`pressure_shape=""`/`pressure_n=0`）。
- **将来要启用**：在透传层把 >12 的读拆成多个 ≤12 的 RM 事务再合并响应；需先确认本机 O6 有压力传感器（否则每周期可能超时数秒、采集速率骤降）。格式本就未验证（见 session3 §9）。

### 5.3 真机运动链路验证（2026-08-07，用户确认 B 选项）

`--real --enable-motion hand preset` 真机实测：
| 命令 | 结果 | 读回 hand_pos_0..5 |
|------|------|---------------------|
| `open`（第一次） | `[ok]` 未动 | `255 254 254 254 254 0`（本就在张开位，无运动可做） |
| `close` | `[ok]` 动了 ✅ | `0 78 58 0 0 0`（半握拳；通道 1/2 停在 78/58 = 机械限位堵转，O6 保护正常） |
| `open`（重试） | `[ok]` 动了 ✅ | `255 254 254 254 254 0`（重新张开，通道 1/2 78/58→254） |

**结论**：O6 → RM75 末端 RS485 透传 → Modbus 写入 → 伺服驱动全链路打通；`--enable-motion` 安全门正确放行；写路径（`rm_write_registers` 字节对契约）与读路径一致，实际驱动成功。

**「通道 5 永远停在 0」**：open（目标 255）和 close（目标 0）下都钉在 0。最可能是通道 5 是特殊关节（如拇指旋转），raw 坐标语义不同——0 是它的伸直/极限位，open 无法再动它。**留作记录，不影响主线。**

---

## 6. 编译坑清单（已修复，重踩即浪费时间）

1. **Recorder 头文件 include `Result` 缺失**：`Recorder.hpp` 用 `Result` 但没 include `robotics/domain/errors/Error.hpp` → 加 include。
2. **Recorder 缺 `pace()` 声明**：`.cpp` 定义了 `pace` 但 header 没声明 → header 补方法。
3. **`Error::make` 签名**：`(ErrorCategory, DeviceType, module, code, message, severity, retry)`——`DeviceType` 不能省（曾写 `make(Internal, "recorder", ...)` 编译错）。
4. **CsvRecordSink metadata 汇总块重复**：`Recorder::stop()` 与 `CsvRecordSink` 析构都会 `flush()` → 汇总块写两遍。修复：加 `summary_written_` 标志，幂等只写一次。
5. **CMake 源列表手写清单**：新增 `src/services/StateCollector.cpp`、`Recorder.cpp`、`src/infrastructure/recording/CsvRecordSink.cpp` 必须显式加入顶层 `robotics_core` 源列表，否则链接不过。
6. **StateCollector include 路径**：`#include "src/services/StateCollector.hpp"` 不存在（头文件在 `include/robotics/services/`）→ 改 `"robotics/services/StateCollector.hpp"`。
7. **`std::to_string_with_precision` 不存在**：`RowBuilder::d()` 直接用 `oss_ << v`（默认 10 位精度）。
8. **pressure 3 层 vector**：`pressure[finger][row][col]`，扁平化需 3 层循环（`[finger][row][col]`）。
9. **RowBuilder `first_` 初始化**：应为 `true`（曾为 `false` 导致每行开头多一空列，表头 111 列）。
10. **CLI `--real` 条件编译**：mock 构建下驱动头不可 include → 用 `HAVE_REALMAN_DRIVER`/`HAVE_LINKERHAND_DRIVER` 宏守卫（apps/CMakeLists 注入），mock 下 `--real` 报「需 debug/hardware 构建」。
11. **`parse_args` 冗余分支**：`--duration=`/`--rate=` 两种写法，`take_next_double` 消费下一个 token 时 `++i` 防重解析。

---

## 7. 阶段4 执行回顾（验收方式）

| 项 | 结果 |
|----|------|
| mock preset 构建+测试 | ✅ 13/13 通过 |
| debug preset 构建+测试 | ✅ 15/15 通过 |
| asan preset 构建+测试 | ✅ 15/15 通过（含修复后 unit_rm_passthrough） |
| `robotctl record` Mock 手工验收 | ✅ 101 帧@50Hz，seq 单调，dropped=0，110 列对齐 |
| `--real` 真实录制（非运动） | ✅ 16~51 帧，arm/hand 均 valid，seq 单调，ASan 无报错，无 SDK 噪音 |
| 真机运动链路（hand preset） | ✅ open/close 双向真实生效 |
| 厂商类型泄漏到上层 | ✅ header 无 rm_*/LinkerHandApi 类型（`void*` + PIMPL） |
| 未验证项（需更多硬件/阶段5） | ⚠ 见 §9 |

---

## 8. 测试覆盖说明

- **`unit_state_store_seq`**：latest 携带统一序号、combined 序号单调、Good（10ms 在阈值内）、Skewed（100ms 超阈值）、Lost（仅 arm）、Unknown（都不在）、**断连视同手断连**（arm 失效后残留 hand 快照按缺席）。
- **`unit_state_collector`**：Mock 双设备 200Hz 连续采集不崩溃 + 单设备（无 hand）不崩溃 + stop 幂等/析构自动 stop。
- **`unit_recorder`**：正常流程（事件入队→落盘→flush）、**序号去重**（store 未更新时只写首帧）、**丢帧**（慢写盘+小队列+尽速采样 → dropped>0 不阻塞）、stop 幂等/析构。
- **`unit_csv_sink`**：表头与行恒 110 列、seq/t_steady 单调、事件与 metadata 落盘、**缺席编码**（hand_valid=0 时数值 0/字符串空）、目录创建/坏目录 is_open=false。
- **`integration_mock_record`**：走 ApplicationService：connect_all → start_collection → record_start → event → sleep → record_stop → 读回 states/events/metadata 断言；重复 record_event 报 ResourceConflict。
- **`unit_rm_passthrough`**（更新）：mock 改为忠实模拟 RM 契约（多读/写 `2*params.num` 个 int8 字节），10 用例全过；写路径断言 `written_values.size()==12`。

---

## 9. 未验证项（阶段5 或后续实测）

| 项 | 说明 |
|----|------|
| `arm home` / `movej` 真实运动 | 阶段4 只实测了手部运动；机械臂运动（`arm_home`/`movej`）未在真机验证，阶段5 轨迹回放时一起测。**每次真机运动仍须用户确认** |
| 通道 5 语义 | open/close 下都停在 0，疑为特殊关节（拇指旋转），raw 坐标语义未明 |
| 压力读取 | 已被跳过（§5.2）；启用需透传层拆分 >12 事务 + 确认本机有压力传感器 |
| O6 透传超时调优 | timeout 当前默认 500ms；若 stop 延迟明显可调小 |
| sync_quality 实测 | 真机记录恒为 Skewed（双采集线程独立采样，时差>50ms）；阶段5 若需 Good 需看是否合意 |

---

## 10. 下一步：阶段5（轨迹复现）

录制的 `states.csv` 即轨迹数据（`seq`/`t_steady_ns` 单调、`arm_j_pos_*`/`hand_pos_*` 列）。阶段5 从 CSV 读回轨迹 → 双设备按时间戳同步回放。

**阶段5 要点**（从 session1 总览 + 用户意图）：
- 轨迹解析：`states.csv` → `std::vector<CombinedRobotState>`（或精简 JointState 序列），复用 CsvRecordSink 的列序。
- 双设备同步回放：arm `movej` + hand `setPosition` 按 `t_steady_ns` 相对时间差下发。
- **单指动作前置需求**（session3 §10 已记）：O6 无单通道 set 方法，阶段7 前需实现「读全通道 → 改单指 → 全量写回」辅助（`read_current_positions()` 已有）。

**阶段5 入口**：`ApplicationService::run_skill()` 仍是占位（返回 Unsupported）；CLI 无 `replay` 命令。RealManAdapter 的 `replay_trajectory()` 只能复现 RM 控制器内部保存的拖动轨迹，**不能**播任意 CSV，且不含 O6——阶段5 需新建 CSV 轨迹回放路径。

---

## 11. 硬性约束（违反即返工）

- 不得编造厂商 API；使用前先查本地头文件（`RM_API2/C++/include/`、`linkerhand-cpp-sdk/include/`）。
- **RM 多寄存器读写 `data` 是 `2*num` 个 int8 原始字节**（§5.1），沿用它，勿再当 num 个 int。
- 每个阶段结束必须可编译、可测试。
- 真实运动默认禁止；每次真实动作须用户确认（阶段4 已确认的范围可复用）。
- 厂商类型/错误码/句柄不得泄漏到上层业务模块（头文件 `void*` + PIMPL）。
- 不允许在厂商 SDK 回调线程中执行耗时业务逻辑。
- 资料缺失时建 TODO + Unsupported 返回值，不得伪造实现伪装成功。

---

## 12. 相关文档索引

| 文档 | 用途 |
|------|------|
| `Prompt.md` | 完整工程规范（总纲） |
| `session1.md` | 任务一总览：阶段0-1 状态、SDK 审计事实 |
| `session2.md` | 阶段2 RM75 适配层执行版 |
| `session3.md` | 阶段3 O6 适配层执行版（三层 Modbus 映射、编译坑） |
| `robot_hand_control/docs/vendor_api_mapping.md` | 厂商 API 逐项映射 |
| `robot_hand_control/docs/implementation_plan.md` | 阶段0-7 计划（阶段4 标 ✅） |
| `robot_hand_control/docs/architecture.md` | 分层架构 |
| `robot_hand_control/docs/threading_model.md` | 线程模型 |
| `robot_hand_control/docs/O6_ModbusRTU_Protocol.md` | O6 寄存器完整映射 |
