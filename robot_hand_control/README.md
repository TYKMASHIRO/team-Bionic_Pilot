# robot_hand_control

RM75-6F 七自由度机械臂 + LinkerHand O6 灵巧手组合控制系统。

> 一期目标：通过拖动示教记录动作轨迹，然后同步复现；核心是机械臂与灵巧手的通信、统一时间戳与协同工作。
> 这是一个可持续扩展的正式工程（后续将扩展座舱操纵杆、油门、按钮、旋钮、视觉识别、Agent 任务调用等），遵循低耦合、可扩展、可测试、可维护、安全原则。

## 快速开始（Mock 模式，无需真实硬件）

```bash
cd robot_hand_control
cmake --preset mock
cmake --build build/mock
ctest --test-dir build/mock          # 运行全部单元/集成测试
./build/mock/bin/robotctl doctor
./build/mock/bin/robotctl status
./build/mock/bin/robotctl arm connect
./build/mock/bin/robotctl hand connect
./build/mock/bin/robotctl arm drag-teach start --record
./build/mock/bin/robotctl arm drag-teach stop
```

> 真实运动默认关闭。所有真实运动命令需显式启用：`robotctl ... --enable-motion`。

## 构建预设

| 预设 | 说明 |
|------|------|
| `mock` | 仅 Mock 设备，无厂商 SDK 依赖，离线可构建/测试 |
| `debug` | 开启 RM75 + O6 驱动 |
| `release` | Release 构建 |
| `asan` | AddressSanitizer |
| `hardware` | 硬件测试（需真实设备） |

```bash
cmake --preset debug
cmake --build build/debug
```

## 目录结构

```
robot_hand_control/
├── apps/robotctl         CLI（只做参数解析与服务调用）
├── include/robotics/     领域类型 + 抽象接口
├── src/                  服务实现（config/logging/services/safety/trajectory/...）
├── drivers/realman       RM75 适配（阶段2）
├── drivers/linkerhand    O6 适配 + RmPassthroughModbus（阶段3）
├── drivers/mock          Mock 设备（无硬件测试基础）
├── config/               YAML 配置示例
├── docs/                 架构/厂商映射/实施计划/硬件设置
├── tests/                单元/集成/硬件测试
└── tools/                辅助工具
```

## 通信链路

```
PC ──TCP/IP(192.168.1.18:8080)── RM75 控制器 ──末端 RS485── LinkerHand O6 (0x27)
```

O6 通过 RM75 控制器的 Modbus RTU 透传功能访问（波特率 115200, 8N1）。详见 `docs/hardware_setup.md` 与 `docs/vendor_api_mapping.md`。

## 文档

- [架构设计](docs/architecture.md)
- [厂商 API 映射](docs/vendor_api_mapping.md)
- [实施计划](docs/implementation_plan.md)
- [硬件设置](docs/hardware_setup.md)
- [O6 Modbus RTU 协议](../../docs/O6_ModbusRTU_Protocol.md)

## 设计原则

- 厂商 SDK 类型（`rm_*_t`、`LinkerHandApi`）不泄漏到上层。
- 机械臂与灵巧手模块互不依赖；协同仅由上层 Skill/Task 完成。
- 所有运动命令经 CommandScheduler 串行化；真实运动需显式启用。
- SafetySupervisor 独立检查新鲜度/限位/力/温度/通信/权限。
- 轨迹数据不硬编码在 C++ 中；动作逻辑与动作数据分离。
