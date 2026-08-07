# session2.md — 阶段二：RM75 适配层（RealManAdapter）

> 本文件是**任务一（阶段二）专用工作 Prompt**。下次涉及阶段2 或继续阶段3 前，先读本文件，再决定下一步动作。
> 它保存了阶段2 的**交付状态、Domain↔RM API 映射、编译坑清单与架构决策**，避免重复劳动与重踩。
> 主文档：`Prompt.md`；任务一总览：`session1.md`；本文件是阶段2 的执行聚焦版。

---

## 1. 阶段2 目标

为 RM75-6F 机械臂实现**厂商隔离适配层** `RealManAdapter`：

1. 实现 `domain::IRobotArm` 全接口（连接/使能/状态/运动/停止/拖动示教/轨迹复现/坐标系/力/诊断）。
2. **厂商类型零泄漏**：`rm_*_t`、`rm_robot_handle*` 只出现在 `drivers/realman/src/` 内部，头文件对上层完全透明。
3. RM 返回码统一转 `domain::Error` / `domain::Result`（`RmErrorMap`）。
4. 提供无硬件单测（错误映射）+ 硬件连接测试（默认 DISABLED）。

阶段2 只做**适配与映射**，不做采集/轨迹/协同（那是阶段4-6）。

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`dev/cyb`（个人同步分支）
- 已提交：`4b5455d`（session1 状态更新）→ `ee41ccb`（阶段2 完成）→ `fc7a549`（阶段0+1）

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成（docs/ 5 份文档） |
| 阶段1 工程骨架 | ✅ 完成 |
| 阶段2 RM75 Adapter | ✅ **完成**（`ee41ccb`，三 preset 构建+测试全过） |
| 阶段3-7 | ⏳ 未开始（下一步：阶段3 O6 Adapter） |

**阶段2 交付物（commit `ee41ccb`，8 文件 +692/-79）**：
- `drivers/realman/include/RealManAdapter.hpp` — PIMPL 头（无厂商类型）
- `drivers/realman/src/RealManAdapter.cpp` — 全部 rm_* 调用
- `drivers/realman/include/RmErrorMap.hpp` / `src/RmErrorMap.cpp` — RM 返回码 → 统一 Error/Result
- `tests/unit/rm_error_map_test.cpp` — 无硬件单测（8 用例）
- `tests/hardware/hw_arm_connect_test.cpp` — 硬件连接测试（DISABLED）
- `drivers/realman/CMakeLists.txt`、`tests/CMakeLists.txt` — 构建接入

---

## 3. 构建命令（每次开工先跑）

```bash
cd robot_hand_control
# Mock 模式（无硬件，最快回归）——阶段2 无关驱动默认 OFF
cmake --preset mock && cmake --build build/mock -j$(nproc) && ctest --test-dir build/mock
# Debug（含 realman/linkerhand 驱动）
cmake --preset debug && cmake --build build/debug -j$(nproc) && ctest --test-dir build/debug
# 硬件测试（编译验证适配层能链接 RM SDK；测试本身 DISABLED 不跑）
cmake --preset hardware && cmake --build build/hardware -j$(nproc) && ctest --test-dir build/hardware
```

**阶段2 验收基线**：`mock`(7) / `debug`(8) / `hardware`(8+1 DISABLED) 全部通过。

---

## 4. Domain 接口 ↔ RM API2 映射（阶段2 核心知识，直接使用）

> RM 侧 API2 v1.1.6 事实详见 `session1.md §4.1`；以下是与 `IRobotArm` 的逐项对应。**禁造 API**，用前查 `RM_API2/C++/include/`。

### 4.1 连接/生命周期

| IRobotArm | RM API2 | 备注 |
|-----------|---------|------|
| `connect()` | `rm_init(RM_TRIPLE_MODE_E)` + `rm_create_robot_arm(ip, port)` | 句柄 id>0 成功，nullptr 失败；线程模式 2=三线程 |
| `disconnect()` | `rm_delete_robot_arm(h)` | 不调 `rm_destroy()`（SDK 实例复用）|
| `is_connected()` | `atomic<bool> connected_` | 本地标志 |

### 4.2 状态

| IRobotArm | RM API2 | 备注 |
|-----------|---------|------|
| `get_state()` | `rm_get_arm_all_state` + `rm_get_current_arm_state` + `rm_get_force_data` | 见下方单位转换 |
| `health_check()` | `rm_get_arm_software_info` | 版本/固件/健康级别 |
| `get_force_torque()` | `rm_get_force_data` | force_data[0..2]=力, [3..5]=力矩 |

**get_state() 单位/推断约定**（阶段2 已定，阶段4 采集沿用）：
- 关节角：**°→rad**（`rm_joints_to_rad`）；`velocity` 留 0（拉模式无速度数组）
- 电流：**mA→A**（`/1000.0`）；温度 ℃ 原样
- 使能：`joint_en_flag[i] != 0`；错误码：`joint_err_code[i]`
- TCP 位姿：**m + rad**（欧拉角 `cur.pose.euler.*`）
- 运动状态推断：`system_errors` 空→`Idle`，否则→`Stopped`；`reached_target` 同推断
- `timestamp` 用 `domain::make_timestamp()`；`sequence=0`（阶段4 由 StateStore 统一打序号）

### 4.3 运动/停止（真实运动须 SafetySupervisor 授权）

| IRobotArm | RM API2 | 参数 |
|-----------|---------|------|
| `move_joint()` | `rm_movej(h, float j[7], v, 0, 0, block)` | 速度比→`v∈[1,100]` |
| `move_pose()` | `rm_movej_p(h, rm_pose_t, v, 0, 0, block)` | 关节空间位姿运动 |
| `move_linear()` | `rm_movel(h, rm_pose_t, v, 0, 0, block)` | 直线 |
| `stop()` | `rm_set_arm_slow_stop(h)` | 缓停 |
| `emergency_stop()` | `rm_set_arm_stop(h)` | 急停，轨迹不可恢复 |

### 4.4 拖动示教 + 轨迹复现（任务一核心）

| IRobotArm | RM API2 |
|-----------|---------|
| `start_drag_teach(record)` | `rm_start_drag_teach(h, record?1:0)` |
| `stop_drag_teach()` | `rm_stop_drag_teach(h)` |
| `trajectory_to_origin(block)` | `rm_drag_trajectory_origin(h, block?1:0)`（20% 速度回起点，复现前必调）|
| `replay_trajectory(block)` | `rm_run_drag_trajectory(h, block?1:0)` |
| `pause_trajectory()` | `rm_pause_drag_trajectory(h)` |
| `continue_trajectory()` | `rm_continue_drag_trajectory(h)` |
| `stop_trajectory()` | `rm_stop_drag_trajectory(h)` |

### 4.5 坐标系 / 使能 / 诊断

| IRobotArm | RM API2 | 备注 |
|-----------|---------|------|
| `set_tool_frame()` | `rm_set_manual_tool_frame(h, rm_frame_t)` | 名称/位姿/载荷 |
| `set_work_frame()` | `rm_set_manual_work_frame(h, name, pose)` | |
| `enable()` | —（RM 连接即使能） | 语义保留，返回 require_connected() |
| `disable()` | —（RM 无 disable 语义） | 返回 ok |
| `clear_error()` | `rm_clear_system_err(h)` | |

### 4.6 RM 返回码 → 统一 Error（RmErrorMap，已单测锁定）

`0成功 / 1参数或状态错误 / -1发送失败 / -2接收超时 / -3解析失败 / -4到位校验或四代不支持 / -5单线程超时 / -6停止规划 / -7三代不支持`。

映射（`rm_to_error` 返回 `Error`；`rm_to_result` 返回 `Result`）：
- 0 → `Error::ok()` / `Result::ok()`
- 1 → `Validation`(1001) Warning retryable
- -1 → `Communication`(1002) Error retryable
- -2 → `Timeout`(1003) Error retryable
- -3 → `Protocol`(1004) Error
- -4 → `Motion`(1005) Error
- -5 → `Timeout`(1006) Error
- -6 → `Cancelled`(1007) Info（停止规划）
- -7 → `Unsupported`(1008) Error
- 其他 → `Internal`(1099) Error

---

## 5. 阶段2 架构决策（已定，不要推翻）

1. **PIMPL 隔离厂商类型**：`RealManAdapter` 头文件不含任何 `rm_*`；`Impl` 持有 `rm_robot_handle*` 与互斥锁。头文件不前置声明 `rm_robot_handle`（详见 §6 编译坑）。
2. **线程安全**：连接生命周期与状态读取共用 `std::mutex`；`connected_` 用 `atomic<bool>`。
3. **速度比约定**：接口 `speed_ratio` 是 0-1 的 double → 映射为 RM 的 `v∈[1,100]`（`std::clamp`）。
4. **block 语义**：接口 `block` 直通 RM 的 `traj_connect` 之后的 block 参数（0 非阻塞/1 阻塞）。
5. **错误传播**：运动/示教类命令返回 `Result`；状态读取失败返回 `valid=false` 的 state（不抛异常）。
6. **enable/disable 空实现**：RM API 无独立使能语义，接口保留以对齐 Mock，返回 require_connected()/ok。
7. **build 集成**：`realman_driver` 链接 `robotics_core`（PUBLIC）+ 导入库 `realman_api`（PRIVATE）+ BUILD_RPATH 指向 `linux_x86_c++_v1.1.6`（运行时能找到 `libapi_cpp.so`）。
8. **硬件测试不碰真实设备**：`hw_arm_connect` 注册进 ctest 但 `DISABLED TRUE`，需显式 `ctest -R hw_arm_connect` 才跑。

---

## 6. 编译坑清单（已修复，重踩即浪费时间）

> 以下错误在 `ee41ccb` 已全部修复。若阶段3 触碰 RealManAdapter，注意别回退这些。

1. **`rm_robot_handle` 前置声明与 `rm_define.h` 冲突**
   - 症状：`conflicting declaration 'typedef struct rm_robot_handle rm_robot_handle'`（rm_define.h:956）
   - 原因：`rm_define.h` 里是 `typedef struct { int id; } rm_robot_handle;`（匿名结构体 typedef），头文件再写 `typedef struct rm_robot_handle rm_robot_handle;` 就是重复定义。
   - 修复：**删除头文件前置声明**。PIMPL 下头文件本来就不需要厂商类型，`Impl` 内部直接用即可。

2. **`rm_to_error` 返回 `Error` 但调用方要 `Result`**
   - 症状：15 处 `could not convert 'Error' to 'Result'`。
   - 修复：新增 `rm_to_result(rc, device, module, op)`：`rc==0→Result::ok()`，否则 `Result::fail(rm_to_error(...))`。所有 `return rm_to_error(...)` 改 `return rm_to_result(...)`。注意 `connect()` 里的 `Result::fail(rm_to_error(...))` 用法保留不变。

3. **`using namespace robotics::domain` 作用域**
   - 症状：`rm_to_result` 里 `Result` 未声明。
   - 原因：`using namespace` 写在 `rm_to_error` **函数体内**，不跨函数生效。
   - 修复：`rm_to_result` 内写全 `domain::Result::ok()`。

4. **sign-compare 警告 ×3**
   - `kArmDof` 是 `std::size_t`，循环用 `int i` 比较报警。改 `std::size_t i`。（`rm_joints_to_rad`/`rad_to_rm_joints`/`get_state` 三处。）

5. **硬件 preset 配置失败**
   - 症状：`set_tests_properties Can not find test to add properties to: hw_arm_connect`。
   - 原因：`add_executable(hw_arm_connect ...)` 但**没 `add_test()`** 就 `set_tests_properties`。
   - 修复：`add_test(NAME hw_arm_connect COMMAND hw_arm_connect)`，再 `set_tests_properties(... DISABLED TRUE)`。

6. **硬件测试缺 gtest 链接**
   - 症状：`hw_arm_connect` 只有 `robotics_core`/`mock_devices`，无 `GTest::gtest_main`，链接必失败。
   - 修复：补 `${GTEST_LIB}` 与 `target_include_directories(... ${CMAKE_SOURCE_DIR}/src)`。

---

## 7. 阶段2 执行回顾（验收方式）

| 项 | 结果 |
|----|------|
| debug preset 构建 | ✅ 零错误零警告 |
| `unit_rm_error_map`（无硬件单测）| ✅ 8/8 通过 |
| mock preset 回归 | ✅ 7/7 通过 |
| hardware preset 配置+构建 | ✅ `hw_arm_connect` 二进制生成，ctest 注册 DISABLED |
| 厂商类型泄漏到上层 | ✅ 头文件无 rm_*，`grep` 验证通过 |

---

## 8. 下一步：阶段3（O6 Adapter）

阶段2 只覆盖了**机械臂侧**。任务一的另一半是**灵巧手 O6**（不直连电脑，经 RM75 末端 RS485 Modbus RTU 透传）。

| 子任务 | 要点 |
|--------|------|
| `RmPassthroughModbus : IModbus` | 三层翻译：①解析 O6 SDK 发来的完整帧（slave_id/功能码/地址/数据/CRC）②映射到 RM 高层寄存器 API（0x04→读输入、0x06/0x10→写）③重组 RM 返回值为完整 Modbus RTU 响应帧（含 CRC）回给 O6 SDK |
| `LinkerHandAdapter : IDexterousHand` | 包 `LinkerHandApi(O6, RIGHT, MODBUS)`，注入 ModbusTx/Rx 回调 |
| Mock 传输层单测 | 帧解析/CRC 计算/读写映射，无硬件 |
| 关键事实 | O6 从站地址 0x27（右手）；baudrate 115200；RM 透传 port=1（末端接口板）；`rm_read_multiple_input_registers` 的 num 范围 3~12 |

O6 侧 SDK 事实见 `session1.md §4.2`；Modbus 寄存器映射见 `docs/O6_ModbusRTU_Protocol.md`。

---

## 9. 硬性约束（违反即返工）

- 不得编造厂商 API；使用前先查本地头文件（`RM_API2/C++/include/`、`linkerhand-cpp-sdk/include/`）。
- 每个阶段结束必须可编译、可测试。
- 真实运动默认禁止；未配置的安全项默认拒绝运动。
- 厂商类型/错误码/句柄不得泄漏到上层业务模块。
- 轨迹点不得硬编码在 C++ 源文件中。
- 日志写盘/轨迹保存不得阻塞设备状态采集线程。
- 不允许在厂商 SDK 回调线程中执行耗时业务逻辑。
- 资料缺失时建 TODO + Unsupported 返回值，不得伪造实现伪装成功。

---

## 10. 相关文档索引

| 文档 | 用途 |
|------|------|
| `Prompt.md` | 完整工程规范（总纲） |
| `session1.md` | 任务一总览：阶段0-1 状态、SDK 审计事实（RM §4.1 / O6 §4.2 / Modbus 透传 §4.3）、核心架构矛盾 |
| `robot_hand_control/docs/vendor_api_mapping.md` | 厂商 API 逐项映射 |
| `robot_hand_control/docs/architecture.md` | 分层架构 |
| `robot_hand_control/docs/implementation_plan.md` | 阶段0-7 计划 |
| `robot_hand_control/docs/threading_model.md` | 线程模型 |
| `robot_hand_control/docs/O6_ModbusRTU_Protocol.md` | O6 寄存器完整映射（阶段3 必读） |
