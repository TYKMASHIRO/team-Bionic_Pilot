# 实施计划 (Implementation Plan)

> 对应 Prompt.md 第十八节。每个阶段结束时必须可编译、可测试。

## 阶段总览

| 阶段 | 名称 | 核心交付 | 状态 |
|------|------|----------|------|
| 0 | 仓库与 SDK 审计 | 4 份 docs | ✅ 完成 |
| 1 | 工程骨架 | CMake、领域类型、接口、错误、配置、日志、Mock、最小 CLI、单测 | ✅ 完成 |
| 2 | RM75 Adapter | RealManAdapter：只读 + 低速运动 + 拖动示教接口 | ✅ 完成 |
| 3 | O6 Adapter | LinkerHandAdapter + RmPassthroughModbus | ✅ 完成 |
| 4 | 统一状态与同步记录 | StateStore、Recorder、事件、元数据 | 待开始 |
| 5 | 轨迹管理与复现 | 加载/校验/调速/Dry-run/Mock 复现 | 待开始 |
| 6 | Skill 框架 | 7 个基础 Skill + Runtime | 待开始 |
| 7 | 座舱动作 Skill | 6 个 cockpit Skill | 待开始 |

## 阶段 0：仓库和 SDK 审计（✅）

已完成：
- 定位 RM75 API2（v1.1.6）头文件、库、ModbusRTU 示例。
- 定位 O6 SDK（v2.0.0）头文件、库、Modbus 示例。
- 确认拖动示教/轨迹录制 API（`rm_start_drag_teach`/`rm_run_drag_trajectory` 等）。
- 确认 O6 SDK 回调注入模式与 Modbus 传输层接口。
- 产出：`architecture.md`、`vendor_api_mapping.md`、`implementation_plan.md`、`hardware_setup.md`。

待确认（硬件实测后更新 vendor_api_mapping.md）：
- O6 压力传感器数据格式。
- RM75 末端 RS485 引脚定义（图片资料待查看）。
- O6 经 RM 透传的响应延迟与超时。

## 阶段 1：工程骨架

**任务**：
1. 建立 CMake 工程（顶层 + drivers 子目录），CMakePresets，编译选项模块。
2. 统一数据类型：`Timestamp/JointVector/Pose/ForceTorque`、`RobotArmJointState/RobotArmState/DexterousHandState/CombinedRobotState`、`DeviceHealth/CommandResult/Error`。
3. 抽象接口：`IRobotArm/IDexterousHand/IClock/IStateStore/ISafetySupervisor/ITrajectoryRepository/IRecordSink/ISkill`。
4. 错误模型：统一错误分类、设备类型、严重程度、可重试。
5. 配置模块：YAML 解析 + 校验（robot/safety/logging）。
6. 日志模块：结构化日志。
7. Mock 设备：`MockRobotArm`/`MockDexterousHand`。
8. 最小 CLI：`robotctl doctor` / `robotctl status`。
9. 单元测试：Mock 连接、状态快照、安全拒绝、命令取消、非法配置。

**验收**：
- 无厂商 SDK 也可编译（用 `ENABLE_REALMAN_DRIVER=OFF` 等开关）。
- `robotctl doctor` 可运行。
- 所有单元测试通过。

## 阶段 2：RM75 Adapter

**任务**：
- 连接/断开（`rm_init(RM_TRIPLE_MODE_E)` + `rm_create_robot_arm`）。
- SDK 版本读取（`rm_api_version`）。
- 状态读取（拉模式 `rm_get_arm_all_state` + `rm_get_current_arm_state` + `rm_get_force_data`）。
- 七关节数据转换（`float[7]`，°→rad）。
- TCP 位姿（`rm_pose_t`：m + rad/quat）。
- 六维力（`rm_force_data_t`）。
- 停止（`rm_set_arm_slow_stop`/`rm_set_arm_stop`）。
- 错误转换（RM int 错误码 → 统一 Error）。
- 拖动示教接口（`rm_start_drag_teach`/`rm_stop_drag_teach`）。
- 无运动硬件连接测试（Vendor Adapter Test，不运动）。

**顺序**：先只读能力，再加低速运动能力。真实运动默认关闭。

## 阶段 3：O6 Adapter（✅ 完成）

**任务**：
- `RmPassthroughModbus : IModbus` 传输层（★ 关键）：
  - 实现 O6 SDK 回调所需收发能力，底层调用 RM 寄存器 API。
  - 解析 O6 完整 Modbus RTU 帧 → 映射 RM API（0x04 读输入、0x03 读保持、0x06 写单寄存器、0x10 写多寄存器）→ 重组响应帧。
  - 从站地址 0x27，port=1（末端 RS485），波特率 115200。
- `DirectSerialModbus : IModbus` 占位（接口 + 未实现，不伪造）。
- `LinkerHandAdapter : IDexterousHand`：
  - 构造 `LinkerHandApi(O6, RIGHT, MODBUS)` + 回调注入。
  - 六通道位置控制（0-255 raw）。
  - 状态读取（位置/速度/转矩/温度/故障/版本）。
  - 张开 `{255×6}` / 闭合 `{0×6}` / 预抓取 `{255,128,255,255,255,255}`。
  - 错误转换。
- 连接顺序：先连 RM75，再配置透传，再访问 O6。

**验收**：Mock 传输层可单测帧解析/组帧/CRC；硬件测试显式开启。

**完成记录（详见 `session3.md`）**：
- 新增 `linkerhand_codec`（纯函数 CRC16/帧解析/组帧，无厂商依赖，mock preset 可构建可测）与 `linkerhand_driver`（受 `ENABLE_LINKERHAND_DRIVER` 控制）。
- `RmPassthroughModbus`：Tx/Rx 分离回调（`sendRawFrame` 执行 RM 事务并缓存响应，`receiveCompleteFrame` 取缓存）；单读 1 / 多读 3~12 / 多写 ≤10 数量限制映射；0x10 写后不回读（O6 只支持 0x04 读，回读校验不可靠）；header 用 `void*` 隔离 `rm_robot_handle`。
- `LinkerHandAdapter`：预设 Open/Close/PreGrasp，`stop()` 重发当前位置，`clearError` 返回 Unsupported。
- 测试：`unit_modbus_frame_codec`（11 用例，mock/debug/hardware 均跑）；`unit_rm_passthrough`（`-Wl,--wrap=` 模拟 rm_* 符号，仅 debug/hardware）。
- 结果：mock 8/8、debug 10/10、hardware 10/10 全部通过；`hw_arm_connect` 保持 DISABLED。
- 未验证项（需硬件）：RM75 透传时序/波特率、O6 实际响应帧、压力数据格式、`rm_write_registers` 行为。

**阶段4 入口**：`IDexterousHand` 已由 `LinkerHandAdapter` 满足，`LinkerHandAdapter::get_state()` 返回 `DexterousHandState` 可直接接入 `StateStore`。

## 阶段 4：统一状态与同步记录

**任务**：
- StateStore（不可变快照，线程安全）。
- 统一时间戳（`IClock`：steady + system）。
- CombinedRobotState（双设备 + 时间差 + 同步质量）。
- Recorder（有界队列 + 后台写盘线程 + 丢帧统计 + 事件标记）。
- 录制元数据（session_id、版本、配置 hash、标定）。

**验收**：Mock 双设备连续记录；单设备离线不崩溃；文件可读；时间戳单调；字段完整。

## 阶段 5：轨迹管理与复现

**任务**：
- 轨迹格式（统一时间轴 + 状态 + 事件）。
- 加载、格式/版本/关节数/时间戳单调/NaN/范围校验。
- 调速、重采样、起点对齐。
- Dry-run 校验（起点偏差、设备在线、安全许可）。
- Mock 复现、取消、超时、执行报告。

**验收**：Mock 复现完全通过前不允许真实组合运动。

## 阶段 6：Skill 框架

**Skill 清单**：
- `arm.move_to_safe_pose`
- `arm.drag_teach_record`（拖动示教 + 同步记录双设备）
- `hand.open` / `hand.close` / `hand.apply_preset`
- `combined.synchronized_replay`（★ 双设备协同复现）
- `combined.safe_release`

每个 Skill：manifest + 参数 + 前置条件 + 状态机 + 结果 + 测试。

## 阶段 7：座舱动作 Skill

- `cockpit.approach_control_stick` / `grasp_control_stick` / `release_control_stick`
- `cockpit.push_throttle` / `press_button` / `rotate_knob`

用阶段、成功判据、超时、安全策略、恢复策略实现，不得写成无状态 API 调用串。

## 构建与测试

- CMakePresets：debug/release、BUILD_TESTING、ENABLE_REALMAN_DRIVER、ENABLE_LINKERHAND_DRIVER、ENABLE_HARDWARE_TESTS、ENABLE_SANITIZERS、BUILD_TOOLS。
- 测试框架：GoogleTest（FetchContent 或系统包）。
- Mock 模式编译：两个驱动开关全 OFF。
