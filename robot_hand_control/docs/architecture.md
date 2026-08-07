# robot_hand_control 架构设计

> 对应 Prompt.md 第三节「目标架构」。
> 本文档描述 RM75-6F 机械臂 + LinkerHand O6 灵巧手组合控制系统的分层架构。

## 1. 总体分层

```
┌───────────────────────────────────────────────┐
│ CLI / GUI / Agent / Behavior Tree / ROS2 扩展 │   ← 表示层（一期只有 CLI）
└──────────────────────┬────────────────────────┘
┌──────────────────────▼────────────────────────┐
│ Application API                              │   ← apps/robotctl：参数解析 + 服务调用
│  命令提交、状态查询、任务取消、结果返回        │
└──────────────────────┬────────────────────────┘
┌──────────────────────▼────────────────────────┐
│ Task Orchestration                           │   ← include/robotics/orchestration
│  完整任务编排、状态机、组合流程                │
└──────────────────────┬────────────────────────┘
┌──────────────────────▼────────────────────────┐
│ Skill Runtime                                │   ← include/robotics/skills
│  技能注册、执行、取消、资源仲裁、结果管理       │
└───────────────┬──────┴───────────────┬────────┘
┌───────────────▼────────┐ ┌───────▼──────────────┐
│ IRobotArm              │ │ IDexterousHand       │   ← 领域接口（domain interfaces）
│ 机械臂能力抽象          │ │ 灵巧手能力抽象        │
└───────────────┬────────┘ └───────┬──────────────┘
┌───────────────▼────────┐ ┌───────▼──────────────┐
│ RealManAdapter         │ │ LinkerHandAdapter    │   ← drivers/
│ RM API2 适配           │ │ O6 SDK/Modbus 适配    │
└───────────────┬────────┘ └───────┬──────────────┘
┌───────────────▼────────┐ ┌───────▼──────────────┐
│ RM75-6F                │ │ O6                   │   ← 硬件
└────────────────────────┘ └──────────────────────┘
```

依赖方向严格单向：

```
Application → Task → Skill → Domain Interfaces → Vendor Adapters → Vendor SDK / Protocol
```

禁止反向依赖和跨层调用。机械臂模块不得依赖灵巧手模块，灵巧手模块不得依赖机械臂模块；两台设备协同只能由上层 Skill/Task 完成。

## 2. 横向公共模块

| 模块 | 职责 | 位置 |
|------|------|------|
| SafetySupervisor | 安全监督：新鲜度/限位/力/温度/通信 | `include/robotics/safety/ISafetySupervisor.hpp` |
| StateStore | 统一状态中心（不可变快照） | `include/robotics/interfaces/IStateStore.hpp` |
| CommandScheduler | 命令调度：串行化、生命周期、取消/超时 | `include/robotics/services/` |
| Recorder | 同步动作记录（后台写盘线程） | `include/robotics/interfaces/IRecordSink.hpp` |
| TrajectoryManager | 轨迹加载/校验/调速/复现 | `include/robotics/trajectory/` |
| SkillRegistry | 技能注册与描述 | `include/robotics/skills/` |
| ResourceManager | 设备资源互斥 | `include/robotics/services/` |
| Configuration | 配置管理（YAML 校验） | `src/infrastructure/config/` |
| Diagnostics | 诊断与 doctor | `apps/diagnostics/` |
| Logging | 结构化日志 | `src/infrastructure/logging/` |
| Clock | 统一单调时钟 | `include/robotics/interfaces/IClock.hpp` |
| Storage | 轨迹与元数据存储 | `src/infrastructure/storage/` |
| MockDevices | 无硬件测试设备 | `drivers/mock/` |

## 3. 核心设计决策

### 3.1 厂商 SDK 隔离

- 厂商类型（`rm_*_t`、`rm_robot_handle*`、`LinkerHandApi`、Modbus 寄存器结构）**只允许出现在 `drivers/` 内部**。
- 上层只能看到统一领域类型：`RobotArmState`、`DexterousHandState`、`Pose`、`JointVector`、`ForceTorque`、`DeviceHealth`、`CommandResult`、`Error`。
- 对应关系记录在 `docs/vendor_api_mapping.md`。

### 3.2 设备抽象

- `IRobotArm`：连接、使能、运动、拖动示教、力、坐标系、健康检查。
- `IDexterousHand`：连接、关节控制、预设、状态、健康检查。
- 两者通过 `IStateStore` 统一快照，通过 `IClock` 统一时间戳。

### 3.3 O6 通信链路（当前唯一路径）

```
PC ──TCP/IP── RM75 控制器 ──末端 RS485── O6
```

O6 通过 RM75 的 Modbus RTU 透传功能访问：

1. 先连接 RM75（`rm_create_robot_arm`）。
2. 配置末端 RS485 为 Modbus RTU 主站：`rm_set_modbus_mode(handle, port=1, baudrate=115200, timeout)`。
3. O6 通过 RM 的高层 Modbus 寄存器 API 读写。

**关键点（审计结论）**：RM 的 Modbus 透传是**高层功能码封装**，不是原始字节透传：

| RM API | 功能 | 限制 |
|--------|------|------|
| `rm_read_input_registers` | 读输入寄存器（0x04） | 单次仅 1 个寄存器 |
| `rm_read_multiple_input_registers` | 读多个输入寄存器 | 2<num<13（3~12 个） |
| `rm_read_holding_registers` | 读保持寄存器（0x03） | 单次仅 1 个 |
| `rm_read_multiple_holding_registers` | 读多个保持寄存器 | 2<num<13 |
| `rm_write_single_register` | 写单个寄存器（0x06） | - |
| `rm_write_registers` | 写多个寄存器（0x10） | num≤10 |

而 O6 SDK（`LinkerHandApi`）的 Modbus 回调模式要求**收发完整 Modbus RTU 帧**（含从站地址、功能码、寄存器地址、数据、CRC）。

**架构决策**：`RmPassthroughModbus` 传输层实现 `linkerhand::communication::IModbus` 接口，在内部：
1. **解析** O6 SDK 构造的完整 Modbus RTU 帧（slave_id、function code、reg_addr、data、CRC）。
2. **映射**到 RM 的高层寄存器 API 调用（按功能码分发：0x04→读输入、0x03→读保持、0x06→写单寄存器、0x10→写多寄存器）。
3. **重组** RM 返回的寄存器值为 Modbus RTU 响应帧（含 CRC 校验与生成）。

这样 O6 SDK 上层（`LinkerHandApi`）完全无感知，不需要任何代码修改即可经由 RM 透传工作。

> 注意：RM 的 Modbus 从站地址通过 `rm_peripheral_read_write_params_t.device` 指定（O6 右手 = 0x27），`port=1` 表示末端接口板 RS485。

### 3.4 时间戳模型

- **控制时序**用 `std::chrono::steady_clock`（单调、不受系统时间调整影响）。
- **日志/文件名**用 `std::chrono::system_clock`。
- 两者不得混用。`IClock` 抽象封装单调时钟，使测试可注入。
- 所有状态快照携带 `Timestamp` 与 `sequence`，用于判断数据新鲜度和做时间同步。

### 3.5 线程模型（概述，详见 threading_model.md）

| 线程 | 职责 | 约束 |
|------|------|------|
| Main/Application | CLI、启停 | 不采集 |
| RM75 采集线程 | 状态拉取/回调→快照入队 | 不写文件、不执行 Skill |
| O6 采集线程 | 状态拉取→快照入队 | 不写文件 |
| CommandScheduler 线程 | 命令串行化、生命周期 | - |
| SafetySupervisor 线程 | 安全检查 | 必要时请求停止 |
| Recorder 写盘线程 | 有界队列消费 | 不阻塞采集 |
| SkillRuntime | Skill 状态机 | 不直接操作串口/SDK |

关键约束：不允许多模块同时向同一设备发运动命令；文件写盘/日志不得持有设备锁；回调中不得等待另一设备；所有线程受控停止；退出顺序 = Skill → 记录 → 运动 → 断开。

### 3.6 命令与状态机

每个命令/Skill 状态机：`Created → Validating → Queued → Running → Succeeded/Failed/Cancelled/TimedOut/SafetyStopped`（含 `Cancelling` 过渡态）。

每条命令含：`command_id`、类型、目标设备、参数、创建时间、截止时间、安全策略、取消令牌、结果。

### 3.7 动作分级

- **L0 设备命令**：单次厂商命令，只存在于 Driver/Adapter 内部。
- **L1 运动原语**：arm 关节/直线运动、hand 关节运动、等待到位、停止。
- **L2 单设备 Skill**：回安全位、进入拖动示教、手张开/闭合/预抓取。
- **L3 组合 Skill**：接近并抓握操纵杆、释放、推油门、按按钮、旋转旋钮。
- **L4 任务**：座舱操作流程、安全释放撤离。

Agent/GUI/CLI 只能调用 L3/L4。不得直接提交关节角和 Modbus 寄存器。

### 3.8 配置管理

YAML 配置（robot/safety/logging），启动时校验类型与范围，计算 hash 写入录制元数据。非法配置在设备连接前失败。所有安全阈值在配置文件中，未配置的安全项默认拒绝真实运动。

### 3.9 Mock 策略

`MockRobotArm` / `MockDexterousHand` 实现与真实设备相同的领域接口，支持连接/断开/运动/到位/超时/超限/故障注入。无硬件时全系统可编译、可测试。Mock 模式下同样运行完整安全逻辑。

## 4. 目录到架构层的映射

| 目录 | 架构角色 |
|------|---------|
| `apps/robotctl` | Application API（CLI） |
| `include/robotics/domain` | 领域类型（类型、命令、状态、错误、结果） |
| `include/robotics/interfaces` | 抽象接口（IRobotArm、IDexterousHand、IClock、IStateStore 等） |
| `include/robotics/services` | 服务层接口（CommandScheduler、ResourceManager） |
| `include/robotics/safety` | 安全接口 |
| `include/robotics/trajectory` | 轨迹接口 |
| `include/robotics/skills` | Skill 接口 |
| `include/robotics/orchestration` | Task 编排接口 |
| `src/*` | 各层实现 |
| `drivers/realman` | RealManAdapter（RM75 适配） |
| `drivers/linkerhand` | LinkerHandAdapter（O6 适配） |
| `drivers/mock` | Mock 设备 |
| `config/` | YAML 配置与示例 |
| `data/` | 录制/轨迹/Skill/标定数据 |
| `tests/` | 分层测试 |
| `third_party/` | 厂商 SDK 引用 |
| `tools/` | 辅助工具 |
