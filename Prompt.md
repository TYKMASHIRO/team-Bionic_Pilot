你现在是一名负责机器人控制软件架构、C++工程化开发、设备驱动集成、
实时状态采集、轨迹记录与动作复现的高级机器人软件工程师。

请在当前工作区中，为以下设备设计并逐步实现一套可长期维护的C++控制软件：

1. 睿尔曼 RM75-6F 七自由度机械臂；
2. 灵心巧手 LinkerHand O6；
3. RM75-6F与O6组合形成的“机械臂+灵巧手”操作系统。

这不是一次性Demo，而是一套后续要继续扩展座舱操纵杆、油门、按钮、旋钮、
视觉识别、Agent任务调用和轨迹学习能力的正式工程。

必须以“低耦合、可扩展、可测试、可维护、可读、安全”为首要原则。

==================================================
一、当前项目背景
==================================================

当前已知硬件与通信条件：

1. 机械臂型号：RM75-6F（七自由度）；
2. 机械臂通过以太网（TCP/IP）通信，当前地址：**192.168.1.18:8080**；
3. 机械臂SDK：RM_API2 v1.1.6（C++版），位于 `RM_API2/C++/`；
    - Linux x86_64 动态库：`RM_API2/C++/linux/linux_x86_c++_v1.1.6/libapi_cpp.so`
4. 灵巧手型号：LinkerHand O6（**右手**，Modbus ID = 0x27）；
5. O6 通过 Modbus RTU 通信，默认波特率 115200，不可软件修改；
    - O6 通信参数：数据位8 / 停止位1 / 校验位none；
6. O6 物理链路（当前唯一路径）：
    - **电脑 → 以太网 → RM75 控制器 → RM75 末端 RS485 接口 → O6**
    - Modbus RTU 命令通过 RM75 控制器的 Modbus RTU 透传功能转发（参见 `RM_API2/Demo/RMDemo_Cpp/RMDemo_ModbusRTU/`）；
    - O6 Modbus RTU 协议详见：`docs/O6_ModbusRTU_Protocol.md`
7. O6 SDK：LinkerHand C++ SDK v2.0.0，位于 `linkerhand-cpp-sdk/`；
    - Linux x86_64 动态库：`linkerhand-cpp-sdk/lib/linux/x86_64/linkerhand_cpp_sdk.so.2.0.0`
8. O6 已安装固定于 RM75 末端，硬件连通性已验证通过；
9. 开发环境：**Linux Ubuntu 22.04 x86_64**，GCC 11.4，CMake 3.22；
    - IDE：**CLion**（项目根目录直接以 CLion 打开 `team-Bionic_Pilot/` 目录）；
    - 不使用 clang-format/clang-tidy，代码格式化使用 CLion 内置工具；
10. 工程使用 C++17 和 CMake（CMakePresets）；
11. 一期不要求 ROS2，不要求微服务、GUI 或网络分布式架构；
12. 系统必须支持以下运行模式：
    - 仅连接 RM75；
    - 仅连接 O6；
    - RM75 和 O6 组合运行；
    - 无真实设备的 Mock 仿真运行；
13. 当前核心目标是：
    - 稳定控制 RM75；
    - 稳定控制 O6；
    - 同步采集两台设备状态；
    - 记录人工操作产生的动作轨迹；
    - 将轨迹整理为可以重复执行的动作；
    - 将多个动作封装为可复用的 Skill；
    - 后续允许 Agent、行为树、GUI 或其他模块调用 Skill。

不得假设当前仓库中所有 SDK 版本、API 名称、结构体字段和协议寄存器与网络资料完全一致。
本地 SDK 头文件、厂商示例、设备协议文档和实际测试结果是唯一事实来源。

**本地事实来源清单：**
- RM75 SDK 头文件：`RM_API2/C++/include/rm_interface.h`、`rm_define.h`、`rm_service.h`
- RM75 C++ 库：`RM_API2/C++/linux/linux_x86_c++_v1.1.6/libapi_cpp.so`
- O6 SDK 头文件：`linkerhand-cpp-sdk/include/api/LinkerHandApi.h`
- O6 SDK 通用定义：`linkerhand-cpp-sdk/include/core/Common.h`
- O6 Modbus 协议文档：`docs/O6_ModbusRTU_Protocol.md`
- O6 SDK 示例：`linkerhand-cpp-sdk/examples/test_o6_modbus_0.cpp`
- RM75 Modbus RTU 透传示例：`RM_API2/Demo/RMDemo_Cpp/RMDemo_ModbusRTU/`

==================================================
二、你的工作原则
==================================================

必须遵守以下原则：

1. 不得直接根据记忆编造厂商API函数名称；
2. 使用任何RM75或O6接口前，必须先在本地SDK头文件、文档和示例中检索确认；
3. 将实际厂商API与项目抽象接口的对应关系写入：
   docs/vendor_api_mapping.md；
4. 厂商SDK的结构体、枚举、错误码、句柄不得泄漏到上层业务模块；
5. 所有厂商API调用必须封装在对应的Adapter/Driver模块中；
6. 机械臂模块不得直接依赖灵巧手模块；
7. 灵巧手模块不得直接依赖机械臂模块；
8. 两台设备的协同只能由上层Skill或Task模块完成；
9. 轨迹点不得直接硬编码在业务C++源文件中；
10. 动作逻辑和动作数据必须分离；
11. 所有硬件资源使用RAII管理；
12. 不使用裸指针表达所有权；
13. 不使用全局可变单例保存设备状态；
14. 不允许在厂商SDK回调线程中执行耗时业务逻辑；
15. 不允许日志写盘、轨迹保存或配置读取阻塞设备状态采集线程；
16. 不允许Agent、GUI、CLI直接调用厂商SDK；
17. 任何真实运动命令默认处于禁止状态；
18. 必须显式开启真实运动权限后，系统才允许运动；
19. 首次运行、配置异常、状态过期、通信中断时，默认拒绝运动；
20. 所有阶段必须保持工程可编译、可测试、可运行。

不要一开始实现微服务、ROS2、复杂GUI或网络分布式架构。
一期使用“模块化单体架构”，但模块边界要允许以后拆成独立进程。

==================================================
三、目标架构
==================================================

采用以下总体分层：

┌───────────────────────────────────────────────┐
│ CLI / GUI / Agent / Behavior Tree / ROS2扩展 │
└──────────────────────┬────────────────────────┘
│
┌──────────────────────▼────────────────────────┐
│ Application API                              │
│ 命令提交、状态查询、任务取消、结果返回          │
└──────────────────────┬────────────────────────┘
│
┌──────────────────────▼────────────────────────┐
│ Task Orchestration                           │
│ 完整任务编排、状态机、组合流程                  │
└──────────────────────┬────────────────────────┘
│
┌──────────────────────▼────────────────────────┐
│ Skill Runtime                                │
│ 技能注册、执行、取消、资源仲裁、结果管理         │
└───────────────┬──────┴───────────────┬────────┘
│                      │
┌───────────────▼────────────┐ ┌───────▼──────────────┐
│ IRobotArm                  │ │ IDexterousHand       │
│ 机械臂能力抽象              │ │ 灵巧手能力抽象         │
└───────────────┬────────────┘ └───────┬──────────────┘
│                      │
┌───────────────▼────────────┐ ┌───────▼──────────────┐
│ RealManAdapter             │ │ LinkerHandAdapter    │
│ RM API2适配                │ │ O6 SDK/Modbus适配     │
└───────────────┬────────────┘ └───────┬──────────────┘
│                      │
RM75-6F                  O6

横向公共模块：

- SafetySupervisor：安全监督；
- StateStore：统一状态中心；
- CommandScheduler：命令调度；
- Recorder：同步动作记录；
- TrajectoryManager：轨迹管理；
- SkillRegistry：技能注册；
- ResourceManager：设备资源互斥；
- Configuration：配置管理；
- Diagnostics：诊断；
- Logging：结构化日志；
- Clock：统一单调时钟；
- Storage：轨迹和元数据存储；
- MockDevices：无硬件测试设备。

依赖方向必须严格保持为：

Application
↓
Task
↓
Skill
↓
Domain Interfaces
↓
Vendor Adapters
↓
Vendor SDK / Protocol

禁止出现反向依赖和跨层调用。

==================================================
四、推荐工程目录
==================================================

请按以下结构建立工程。可以根据本地SDK实际情况微调，但不能破坏层级边界。

robot_hand_control/
├── CMakeLists.txt
├── CMakePresets.json
├── README.md
├── LICENSE
├── cmake/
│   ├── CompilerOptions.cmake
│   ├── Sanitizers.cmake
│   ├── Dependencies.cmake
│   └── Packaging.cmake
│
├── apps/
│   ├── robotctl/
│   │   └── main.cpp
│   └── diagnostics/
│       └── main.cpp
│
├── include/robotics/
│   ├── domain/
│   │   ├── types/
│   │   ├── commands/
│   │   ├── states/
│   │   ├── errors/
│   │   └── results/
│   │
│   ├── interfaces/
│   │   ├── IRobotArm.hpp
│   │   ├── IDexterousHand.hpp
│   │   ├── IClock.hpp
│   │   ├── IStateStore.hpp
│   │   ├── ISafetySupervisor.hpp
│   │   ├── ITrajectoryRepository.hpp
│   │   ├── IRecordSink.hpp
│   │   └── ISkill.hpp
│   │
│   ├── services/
│   ├── safety/
│   ├── trajectory/
│   ├── skills/
│   └── orchestration/
│
├── src/
│   ├── domain/
│   ├── services/
│   ├── safety/
│   ├── trajectory/
│   ├── skills/
│   ├── orchestration/
│   └── infrastructure/
│       ├── config/
│       ├── logging/
│       ├── storage/
│       └── time/
│
├── drivers/
│   ├── realman/
│   │   ├── include/
│   │   ├── src/
│   │   ├── CMakeLists.txt
│   │   └── README.md
│   │
│   ├── linkerhand/
│   │   ├── include/
│   │   ├── src/
│   │   ├── transport/
│   │   │   ├── RmPassthroughModbus  ← 当前路径：RM75 控制器透传/
│   │   │   └── DirectSerialModbus   ← 扩展路径：电脑 USB-RS485 直连/
│   │   ├── CMakeLists.txt
│   │   └── README.md
│   │
│   └── mock/
│       ├── MockRobotArm.cpp
│       └── MockDexterousHand.cpp
│
├── config/
│   ├── robot.example.yaml
│   ├── safety.example.yaml
│   ├── logging.example.yaml
│   └── skills/
│
├── data/
│   ├── recordings/
│   ├── trajectories/
│   ├── skills/
│   └── calibration/
│
├── docs/
│   ├── O6_ModbusRTU_Protocol.md  ← 已生成：O6 Modbus RTU 寄存器完整映射
│   ├── architecture.md
│   ├── implementation_plan.md
│   ├── vendor_api_mapping.md
│   ├── threading_model.md
│   ├── trajectory_format.md
│   ├── skill_specification.md
│   ├── safety_design.md
│   ├── error_model.md
│   └── hardware_setup.md
│
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── hardware/
│   └── fixtures/
│
├── tools/
│   ├── trajectory_inspector/
│   ├── recording_converter/
│   └── config_validator/
│
└── third_party/
├── realman_api/
└── linkerhand_sdk/

要求：

1. third_party只保存厂商SDK或其引用，不允许业务代码写在其中；
2. drivers只负责厂商接口适配；
3. skills不得包含串口、Socket、SDK句柄和寄存器地址；
4. apps只做参数解析和服务调用，不写控制逻辑；
5. tests中的Mock设备必须能够在没有真实硬件时运行。

==================================================
五、设备抽象边界
==================================================

建立两个核心设备接口。

一、IRobotArm

它代表“机械臂能力”，而不是RM75厂商API。

至少覆盖以下语义能力：

- connect；
- disconnect；
- isConnected；
- enable；
- disable；
- getState；
- moveJoint；
- movePose；
- moveLinear；
- stop；
- emergencyStop；
- startDragTeach；
- stopDragTeach；
- setToolFrame；
- setWorkFrame；
- getForceTorque；
- clearError；
- healthCheck。

二、IDexterousHand

它代表“灵巧手能力”，而不是O6寄存器。

至少覆盖以下语义能力：

- connect；
- disconnect；
- isConnected；
- getState；
- setJointPositions；
- setJointSpeeds；
- setTorqueLimits；
- applyPreset；
- open；
- close；
- stop；
- clearError；
- healthCheck。

这些名称只是领域语义参考。
具体C++接口设计应遵循本地SDK能力，避免设计厂商无法支持的强语义。

厂商API适配要求：

1. RealManAdapter内部处理：
    - RM句柄生命周期；
    - RM错误码转换；
    - 7关节数组转换；
    - 位姿和单位转换；
    - 拖动示教；
    - 六维力读取；
    - 实时状态回调；
    - SDK线程限制；
    - 重连和超时。

2. LinkerHandAdapter内部处理：
    - O6 SDK（LinkerHandApi）实例生命周期；
    - Modbus RTU 通信（当前唯一路径：通过 RM75 控制器 Modbus RTU 透传）；
    - 传输层抽象接口（保留 Direct USB-RS485 扩展能力）；
    - O6 六个控制通道映射（Thumb_Pitch / Thumb_Yaw / Index / Middle / Ring / Little）；
    - 右手配置（Modbus ID 0x27）；
    - 原始值（0–255）与角度/弧度转换；
    - 位置、速度、转矩、温度、故障码和压力传感器反馈；
    - Modbus 寄存器地址映射（详见 `docs/O6_ModbusRTU_Protocol.md`）；
    - 通信错误、重连和超时。

3. 上层只能看到统一类型：
    - RobotArmState；
    - DexterousHandState；
    - Pose；
    - JointVector；
    - ForceTorque；
    - DeviceHealth；
    - CommandResult；
    - Error。

4. 禁止把rm_xxx_t、LinkerHandApi、Modbus寄存器结构直接传到上层。

==================================================
六、统一数据模型
==================================================

请建立明确的数据类型，不允许在业务层大量使用无单位的double数组。

至少定义以下语义类型：

1. Timestamp
    - 使用std::chrono::steady_clock表示控制时序；
    - 使用system_clock时间用于日志和文件名称；
    - 两者不得混用。

2. RobotArmJointState
    - 七个关节位置；
    - 七个关节速度；
    - 七个关节电流；
    - 七个关节温度；
    - 状态有效标志；
    - 数据时间戳；
    - 序号。

3. RobotArmState
    - RobotArmJointState；
    - TCP位置和姿态；
    - 六维力/力矩；
    - 设备错误；
    - 运动状态；
    - 是否到位；
    - 数据新鲜度。

4. DexterousHandState
    - O6六个控制通道的位置；
    - 速度；
    - 电流或扭矩反馈；
    - 温度；
    - 压力数据，如果当前硬件支持；
    - 故障码；
    - 数据时间戳；
    - 序号；
    - 状态有效标志。

5. CombinedRobotState
    - RobotArmState；
    - DexterousHandState；
    - 两者时间差；
    - 同步质量；
    - 当前Skill；
    - 当前命令ID。

6. CommandId
    - 每条命令唯一；
    - 可以追踪提交、接受、开始、完成、失败和取消全过程。

7. Error
    - 统一错误分类；
    - 设备类型；
    - 模块；
    - 错误码；
    - 严重程度；
    - 是否可以重试；
    - 原始厂商错误码；
    - 人类可读说明。

所有单位必须明确。

建议统一：

- 角度：内部使用rad；
- 位置：m；
- 时间：ns或ms；
- 力：N；
- 力矩：N·m；
- 速度和加速度标注单位；
- 厂商单位只允许出现在Adapter内部。

==================================================
七、线程与并发模型
==================================================

一期至少划分以下执行单元：

1. Main/Application线程
    - CLI；
    - 启动和停止系统；
    - 不执行周期采集。

2. RM75状态采集线程
    - 获取或接收机械臂状态；
    - 只做快速转换和入队；
    - 不写文件；
    - 不执行Skill。

3. O6状态采集线程
    - 获取灵巧手状态；
    - 只做快速转换和入队；
    - 不写文件。

4. CommandScheduler线程
    - 串行化同一设备上的运动命令；
    - 管理命令生命周期；
    - 支持取消和超时。

5. SafetySupervisor线程
    - 检查数据新鲜度；
    - 检查关节限制；
    - 检查速度；
    - 检查六维力；
    - 检查通信状态；
    - 必要时请求停止。

6. Recorder后台写盘线程
    - 从有界队列消费记录；
    - 不阻塞设备采集线程；
    - 队列溢出时必须记录丢帧数量。

7. SkillRuntime线程或任务执行器
    - 执行Skill状态机；
    - 不直接操作底层串口或SDK。

并发设计要求：

- 状态数据优先采用不可变快照；
- 共享状态使用明确的同步机制；
- 不允许多个模块同时直接向同一设备发运动命令；
- 设备命令统一经过CommandScheduler；
- 文件写盘不得持有设备锁；
- 日志不得持有设备控制锁；
- 回调中不得等待另一个设备；
- 所有线程都必须支持受控停止；
- 程序退出时必须按顺序停止Skill、停止记录、停止设备运动、断开设备。

将最终线程模型写入docs/threading_model.md。

==================================================
八、命令和状态机设计
==================================================

每条设备命令和Skill至少具有以下状态：

- Created；
- Validating；
- Queued；
- Running；
- Succeeded；
- Failed；
- Cancelling；
- Cancelled；
- TimedOut；
- SafetyStopped。

每条命令至少包含：

- command_id；
- command_type；
- target_device；
- parameters；
- create_time；
- deadline；
- safety_policy；
- cancellation_token；
- result。

不得只用bool表示执行成功或失败。

Skill执行结果至少包含：

- 成功或失败；
- 失败阶段；
- 统一错误；
- 实际执行时间；
- 使用的轨迹版本；
- 使用的配置版本；
- 使用的标定版本；
- 最终设备状态摘要；
- 是否触发安全停止。

==================================================
九、动作与Skill分级
==================================================

建立以下动作分级：

L0：设备命令
- 单次厂商设备命令；
- 只能存在于Driver/Adapter内部。

L1：运动原语
- 机械臂关节运动；
- 机械臂直线运动；
- 灵巧手关节运动；
- 等待到位；
- 停止。

L2：单设备Skill
- 机械臂回安全位；
- 进入拖动示教；
- 灵巧手张开；
- 灵巧手形成预抓取手型。

L3：组合Skill
- 接近并抓握操纵杆；
- 释放操纵杆；
- 推动油门；
- 按压按钮；
- 旋转旋钮。

L4：任务
- 操纵杆接管流程；
- 油门控制流程；
- 起飞前座舱操作流程；
- 异常情况下安全释放和撤离。

Agent、GUI、CLI未来只能调用L3或L4接口。
不得允许Agent直接提交关节角和Modbus寄存器。

每个Skill必须拥有标准化描述：

- id；
- name；
- version；
- description；
- required_resources；
- parameters；
- preconditions；
- execution_stages；
- success_conditions；
- failure_conditions；
- timeout；
- cancellation_policy；
- recovery_policy；
- safety_profile；
- trajectory_references；
- calibration_references。

Skill的结构建议为：

C++负责：
- 执行逻辑；
- 状态判断；
- 超时；
- 异常处理；
- 取消；
- 恢复；
- 安全交互。

YAML/JSON负责：
- 参数；
- 轨迹路径；
- 阈值；
- 速度；
- 版本；
- 前置条件；
- 元数据。

禁止把大量轨迹点写进C++源码。

==================================================
十、动作记录系统
==================================================

实现一个独立Recorder模块。

Recorder必须能够同时记录：

1. 上位机单调时间戳；
2. 系统时间；
3. 记录序号；
4. RM75七关节状态；
5. RM75 TCP位姿；
6. RM75六维力/力矩；
7. RM75错误和运动状态；
8. O6六通道目标值；
9. O6六通道实际反馈；
10. O6电流、温度、压力和故障；
11. 当前下发命令；
12. 当前Skill和Skill阶段；
13. 人工事件标记；
14. 通信状态；
15. 配置版本；
16. 标定版本；
17. SDK版本；
18. 固件版本，如果可以获取。

动作记录分为三类：

A. Raw Recording
- 保留原始状态；
- 不修改；
- 用于问题追溯和数据分析。

B. Processed Trajectory
- 裁剪；
- 重采样；
- 平滑；
- 调速；
- 起点对齐；
- 异常点处理；
- 安全检查。

C. Published Skill Asset
- 已经验证；
- 绑定轨迹版本；
- 可以被SkillRuntime正式执行。

一期存储设计：

1. 先定义IRecordSink；
2. 第一版实现CsvRecordSink或明确的分块文件格式；
3. 元数据使用YAML或JSON；
4. 后续可以增加SQLiteRecordSink或二进制存储；
5. 业务层不得依赖具体文件格式；
6. 写盘使用后台线程；
7. 必须记录丢帧和写盘错误。

建议数据目录：

data/recordings/<session_id>/
├── metadata.yaml
├── combined_state.csv
├── commands.csv
├── events.csv
└── diagnostics.log

data/trajectories/<trajectory_id>/
├── manifest.yaml
├── arm_trajectory.csv
├── hand_trajectory.csv
├── events.csv
└── validation_report.json

metadata至少包含：

- session_id；
- creation_time；
- operator；
- arm_model；
- hand_model；
- arm_sdk_version；
- hand_sdk_version；
- arm_firmware；
- hand_firmware；
- sample_rate；
- calibration_id；
- tool_frame；
- work_frame；
- configuration_hash；
- source_recording；
- processing_history。

==================================================
十一、轨迹复现
==================================================

TrajectoryManager至少负责：

- 加载；
- 格式校验；
- 版本校验；
- 关节数量校验；
- 时间戳单调性校验；
- NaN和无穷值校验；
- 关节范围校验；
- 速度和加速度校验；
- 起始状态校验；
- 调速；
- 重采样；
- 预览；
- Dry-run；
- 正式执行；
- 取消；
- 执行报告。

复现前必须检查：

1. 设备是否在线；
2. 固件和SDK是否兼容；
3. 机械臂是否为七自由度；
4. O6左右手是否匹配；
5. 工具坐标系是否匹配；
6. 标定版本是否匹配；
7. 当前机械臂起点与轨迹起点偏差是否在阈值内；
8. 当前O6姿态与起点是否匹配；
9. 状态数据是否新鲜；
10. 是否存在未清除故障；
11. 安全监督器是否允许执行；
12. 操作者是否已启用真实运动。

组合复现优先采用：

- 单一统一时间轴；
- 状态条件；
- 事件标记；

而不是简单地让机械臂线程和灵巧手线程各自sleep。

例如抓握动作应按阶段执行：

1. 机械臂到达预抓取位置；
2. O6形成预抓取手型；
3. 机械臂低速接近；
4. O6逐步闭合；
5. 根据位置、电流、压力或六维力判断是否抓稳；
6. 满足成功条件后完成；
7. 超时或超力时停止、张开并撤离。

==================================================
十二、安全设计
==================================================

必须建立独立SafetySupervisor。

它不依赖具体Skill，也不能只把安全检查分散写在各个动作内部。

至少检查：

- 设备连接状态；
- 状态数据新鲜度；
- 机械臂七关节软限位；
- TCP工作空间；
- 最大关节速度；
- 最大TCP速度；
- 最大加速度；
- 六维力和力矩阈值；
- O6位置范围；
- O6速度范围；
- O6电流或扭矩阈值；
- O6温度；
- 轨迹起点偏差；
- 通信超时；
- 重复命令；
- 设备资源冲突；
- 配置缺失；
- 标定不匹配；
- 记录器故障是否影响任务；
- 操作者权限；
- 当前是否允许真实运动。

停止级别至少分为：

1. NormalCancel
    - 受控结束当前动作。

2. ControlledStop
    - 尽快停止运动；
    - 保持系统可以恢复。

3. EmergencyStop
    - 调用设备支持的紧急停止机制；
    - 进入锁定状态；
    - 必须人工复位。

重要要求：

- 所有安全阈值放在配置文件中；
- 不得直接使用设备额定最大值作为工作阈值；
- 未配置的安全项默认拒绝真实运动；
- Mock模式下也必须运行相同安全逻辑；
- 必须支持“只校验不执行”的Dry-run模式；
- 真实运动必须使用显式参数启用，例如：
  --enable-motion；
- 高风险动作执行前增加显式确认机制；
- 不得通过日志或软件提示代替硬件急停。

将安全状态机和故障恢复流程写入docs/safety_design.md。

==================================================
十三、配置管理
==================================================

配置使用YAML或JSON，但必须有类型校验和范围校验。

至少包含：

robot.yaml：
- RM75 IP（当前默认：192.168.1.18）；
- RM75 端口（当前默认：8080）；
- RM SDK 工作模式（线程模式：单/双/三线程）；
- O6 通信方式（当前路径：RM75 Modbus RTU 透传 / 可选 Direct USB-RS485）；
- 串口名称（RM75 透传模式下此项不适用；Direct 模式下如 /dev/ttyUSB0）；
- 波特率（当前默认：115200，O6 固件不支持软件修改）；
- 数据位（固定：8）；
- 校验位（固定：none）；
- 停止位（固定：1）；
- Modbus 从站地址（当前默认：0x27 = 右手；左手为 0x28）；
- 左手或右手（当前：右手）；
- 采样频率；
- 自动重连设置。

safety.yaml：
- 关节限制；
- 速度限制；
- 工作空间；
- 六维力限制；
- O6电流限制；
- O6温度限制；
- 状态最大允许延迟；
- 起点偏差；
- 超时；
- 停止策略。

logging.yaml：
- 日志等级；
- 文件目录；
- 滚动大小；
- 保留数量；
- 是否输出控制台；
- 是否记录厂商原始错误。

所有配置：

- 必须提供example文件；
- 不提交真实设备密码或敏感信息；
- 启动时输出配置摘要；
- 输出时隐藏敏感值；
- 计算配置hash并写入记录元数据；
- 非法配置必须在设备连接前失败。

==================================================
十四、日志和诊断
==================================================

使用结构化日志，日志至少包含：

- timestamp；
- level；
- module；
- device；
- command_id；
- skill_id；
- error_code；
- message。

必须实现doctor或diagnostics命令，例如：

robotctl doctor
robotctl status
robotctl devices
robotctl config validate
robotctl arm status
robotctl hand status
robotctl recording list
robotctl trajectory validate <id>
robotctl skill list
robotctl skill inspect <id>

诊断结果应区分：

- 配置错误；
- SDK未找到；
- 动态库未找到；
- 机械臂连接失败；
- 串口打开失败；
- Modbus无响应；
- 设备型号不匹配；
- 固件不兼容；
- 状态数据过期；
- 设备故障；
- 安全限制阻止执行。

==================================================
十五、CLI一期功能
==================================================

至少规划以下命令：

robotctl doctor

robotctl arm connect
robotctl arm status
robotctl arm home --dry-run
robotctl arm stop
robotctl arm drag-teach start
robotctl arm drag-teach stop

robotctl hand connect
robotctl hand status
robotctl hand preset open --dry-run
robotctl hand preset close --dry-run
robotctl hand stop

robotctl record start --name <name>
robotctl record mark --event <event>
robotctl record stop

robotctl trajectory list
robotctl trajectory inspect <id>
robotctl trajectory validate <id>
robotctl trajectory replay <id> --dry-run
robotctl trajectory replay <id> --enable-motion

robotctl skill list
robotctl skill inspect <id>
robotctl skill run <id> --dry-run
robotctl skill run <id> --enable-motion

CLI只负责调用Application Service。
不得在main.cpp里直接访问Driver。

==================================================
十六、Mock和测试策略
==================================================

必须先实现MockRobotArm和MockDexterousHand。

Mock设备需要支持：

- 连接和断开；
- 状态变化；
- 模拟运动；
- 到位；
- 通信超时；
- 状态过期；
- 关节超限；
- 力超限；
- 温度超限；
- 故障码；
- 命令取消；
- 执行超时。

测试分层：

1. Unit Tests
    - 类型转换；
    - 配置校验；
    - 轨迹校验；
    - Skill状态机；
    - 安全判断；
    - 错误码转换。

2. Mock Integration Tests
    - 两台Mock设备组合；
    - 同步记录；
    - 轨迹复现；
    - 取消；
    - 超时；
    - 故障恢复。

3. Vendor Adapter Tests
    - 使用真实SDK但不运动；
    - 版本查询；
    - 连接；
    - 状态读取；
    - 错误转换。

4. Hardware Tests
    - 必须显式开启；
    - 默认不加入普通自动测试；
    - 真实运动测试必须有人工确认。

5. Regression Tests
    - 使用固定录制文件验证轨迹解析；
    - 使用固定故障输入验证安全逻辑。

测试框架优先使用GoogleTest或Catch2。
选择一种，不要同时引入两套。

==================================================
十七、构建和代码质量
==================================================

使用：

- C++17；
- CMake 3.22+；
- CMakePresets；
- **CLion** 作为 IDE（项目根目录 `team-Bionic_Pilot/` 直接以 CLion 打开）；
- CLion 内置代码格式化（不需要额外安装 clang-format/clang-tidy）；
- 编译警告全部开启（`-Wall -Wextra -Wpedantic`）；
- 单元测试；
- 可选 AddressSanitizer 和 UndefinedBehaviorSanitizer。

开发环境：Linux Ubuntu 22.04 x86_64，GCC 11.4。
如果 O6 SDK 当前版本对编译器有特殊限制，必须在文档中记录，并通过独立 Adapter 解决，
不要让编译器限制扩散到整个工程。

编译要求：

- Debug和Release；
- BUILD_TESTING开关；
- ENABLE_REALMAN_DRIVER开关；
- ENABLE_LINKERHAND_DRIVER开关；
- ENABLE_HARDWARE_TESTS开关；
- ENABLE_SANITIZERS开关；
- BUILD_TOOLS开关。

第三方依赖应集中管理。
不得在各子模块中分别下载同一个依赖。

代码风格要求：

- 类型和职责命名明确；
- 单个类只承担单一职责；
- 头文件最小化依赖；
- 优先前置声明；
- 避免巨型Manager；
- 避免上帝对象；
- 避免超过合理长度的函数；
- 禁止catch(...)后静默忽略；
- 错误必须被处理、返回或记录；
- 不使用魔法数字；
- 单位和阈值必须具名；
- public接口必须有简洁文档；
- 复杂并发和安全逻辑必须说明原因。

==================================================
十八、实施阶段
==================================================

不要一次性生成所有代码。
按以下阶段逐步完成，每个阶段结束时必须保持可编译和可测试。

阶段0：仓库和SDK审计

任务：
1. 检查当前目录结构（仓库根：`team-Bionic_Pilot/`）；
2. RM75 SDK 已定位：`RM_API2/C++/`（v1.1.6），头文件在 `include/`，库在 `linux/linux_x86_c++_v1.1.6/`；
3. O6 SDK 已定位：`linkerhand-cpp-sdk/`（v2.0.0），头文件在 `include/api/LinkerHandApi.h`，库在 `lib/linux/x86_64/`；
4. 厂商示例已定位：
    - RM75 Modbus RTU 透传：`RM_API2/Demo/RMDemo_Cpp/RMDemo_ModbusRTU/`
    - O6 Modbus 示例：`linkerhand-cpp-sdk/examples/test_o6_modbus_0.cpp`
5. 编译器：GCC 11.4（Linux x86_64），CMake 3.22，IDE：CLion；
    - RM75 SDK 库：`libapi_cpp.so`（release）、`libapi_cpp_debug.so`（debug）
    - O6 SDK 库：`linkerhand_cpp_sdk.so.2.0.0`
6. 实际 API：
    - RM75：C API（`rm_interface.h`），核心函数 `rm_create_robot_arm(ip, port)`、`rm_init(mode)` 等；
    - O6：C++ API（`LinkerHandApi`），构造函数 `LinkerHandApi(O6, RIGHT, MODBUS)` + 回调注入模式；
7. O6 Modbus RTU 协议已转录：`docs/O6_ModbusRTU_Protocol.md`
    - 输入寄存器 0–87（位置/转矩/速度/温度/故障/版本/压力）
    - 保持寄存器 0–19（控制/速度/转矩/压力选择/波特率）
8. 当前仓库仅包含 SDK 文件 + Prompt.md，尚无业务代码；
9. 风险和缺失资料：
    - O6 压力传感器（华威科）具体数据格式待实测验证；
    - RM75 Modbus RTU 透传的 C++ API 封装方式待确认（需研读 `RMDemo_ModbusRTU` 示例）；
    - 需确认 RM75 末端 RS485 的实际接口引脚定义。

交付：
- docs/vendor_api_mapping.md；
- docs/hardware_setup.md；
- docs/implementation_plan.md；
- docs/architecture.md。

本阶段不执行真实设备运动。

阶段1：工程骨架

任务：
- 建立目录；
- 建立CMake；
- 建立统一数据类型；
- 建立接口；
- 建立错误模型；
- 建立配置；
- 建立日志；
- 建立Mock设备；
- 建立最小CLI；
- 建立单元测试。

验收：
- 无厂商SDK也可以编译Mock版本；
- robotctl doctor可以运行；
- 所有单元测试通过。

阶段2：RM75 Adapter

任务：
- 连接和断开；
- SDK版本读取；
- 状态读取；
- 七关节数据转换；
- TCP位姿；
- 六维力；
- 停止；
- 错误转换；
- 拖动示教接口；
- 无运动的硬件连接测试。

本阶段先完成只读能力，再增加低速运动能力。

阶段3：O6 Adapter

任务：
- O6 SDK（LinkerHandApi）接入，通过 Modbus RTU 回调模式；
- **首要传输层**：RmPassthroughModbus（通过 RM75 控制器 Modbus RTU 透传功能）；
- DirectSerialModbus 建立接口和占位实现，不得伪造未验证 API；
- 参考示例：`linkerhand-cpp-sdk/examples/test_o6_modbus_0.cpp`
- 参考示例：`RM_API2/Demo/RMDemo_Cpp/RMDemo_ModbusRTU/`
- 连接和断开（与 RM75 协同，先连接 RM75 再通过透传通道访问 O6）；
- 版本信息读取；
- 六通道位置控制（0–255 原始值）；
- 状态读取（位置/速度/转矩/温度/故障/压力）；
- 张开（全部 255）和闭合（全部 0）预设；
- 停止；
- 错误转换。

先实现 RmPassthroughModbus（当前唯一可用路径）。
DirectSerialModbus 只建立接口和占位实现，不得伪造未验证 API。

阶段4：统一状态和同步记录

任务：
- StateStore；
- 统一时间戳；
- CombinedRobotState；
- Recorder；
- 事件标记；
- 后台写盘；
- 丢帧统计；
- 录制元数据。

验收：
- Mock双设备连续记录；
- 单设备离线不导致程序崩溃；
- 文件可被工具读取；
- 时间戳单调；
- 状态字段完整。

阶段5：轨迹管理和复现

任务：
- 轨迹格式；
- 轨迹加载；
- 校验；
- 调速；
- Dry-run；
- Mock复现；
- 取消和超时；
- 执行报告。

在Mock复现完全通过前，不允许真实组合运动。

阶段6：Skill框架

先实现以下Skill：

- arm.move_to_safe_pose；
- arm.drag_teach_record；
- hand.open；
- hand.close；
- hand.apply_preset；
- combined.synchronized_replay；
- combined.safe_release。

Skill必须有：
- manifest；
- 参数；
- 前置条件；
- 状态机；
- 结果；
- 测试。

阶段7：座舱动作Skill

逐步实现：

- cockpit.approach_control_stick；
- cockpit.grasp_control_stick；
- cockpit.release_control_stick；
- cockpit.push_throttle；
- cockpit.press_button；
- cockpit.rotate_knob。

不得直接将这些动作写成一串无状态的API调用。
必须使用Skill阶段、成功判据、超时、安全策略和恢复策略。

==================================================
十九、首批验收标准
==================================================

工程至少满足以下验收条件：

1. 在没有真实硬件时，项目可以完整编译；
2. Mock设备可以完成组合Skill；
3. RM75和O6可以分别连接和诊断；
4. 任意一个设备离线不会导致另一个设备模块崩溃；
5. 上层代码不包含厂商SDK类型；
6. 更换O6通信传输层不需要修改Skill；
7. 更换RM机械臂适配器不需要修改O6模块；
8. 轨迹数据不硬编码在C++中；
9. 录制文件包含RM75、O6和统一时间戳；
10. 轨迹执行前执行完整校验；
11. 状态数据过期时拒绝运动；
12. 超力、超温、通信超时可以触发停止；
13. Skill支持取消；
14. Skill失败后返回明确错误和失败阶段；
15. 关键模块有单元测试；
16. 编译无高等级警告；
17. README可以指导新开发者完成Mock构建；
18. docs中包含架构、线程、安全、轨迹和厂商API映射；
19. 真实运动默认关闭；
20. 所有真实运动动作均需要显式启用。

==================================================
二十、Codex每次工作的输出格式
==================================================

每完成一个阶段，请按以下格式汇报：

1. 本阶段完成内容；
2. 修改和新增的文件；
3. 关键架构决策；
4. 执行过的构建命令；
5. 执行过的测试；
6. 测试结果；
7. 尚未验证的厂商接口；
8. 硬件风险；
9. 下一阶段建议。

发现资料缺失时：

- 不要编造API；
- 不要创建假实现伪装成功；
- 可以建立明确的TODO和Unsupported返回值；
- 在docs/vendor_api_mapping.md标出待确认项；
- 在Mock模式继续推进不依赖硬件的工作。

发现现有工程已有可运行代码时：

- 不要直接推翻；
- 先分析现有代码；
- 列出可以保留、封装和重构的部分；
- 小步修改；
- 保证每次修改后仍可编译。

==================================================
二十一、本次立即执行的任务
==================================================

现在先执行阶段0和阶段1，不要直接控制真实设备运动。

具体步骤：

1. 扫描当前工作区目录；
2. 定位RM75 API2、O6 SDK、协议文档和示例；
3. 检查现有C++代码和CMake；
4. 生成当前项目审计报告；
5. 建立docs/architecture.md；
6. 建立docs/vendor_api_mapping.md；
7. 建立docs/implementation_plan.md；
8. 建立基础目录和CMake工程；
9. 建立领域类型和抽象接口；
10. 建立MockRobotArm和MockDexterousHand；
11. 建立最小robotctl doctor/status命令；
12. 建立至少以下测试：
    - Mock连接测试；
    - 状态快照测试；
    - 安全拒绝运动测试；
    - 命令取消测试；
    - 非法配置测试；
13. 编译并运行测试；
14. 汇报本阶段结果。

在阶段0未确认本地厂商API之前，不得实现任何猜测性的RM75或O6调用。