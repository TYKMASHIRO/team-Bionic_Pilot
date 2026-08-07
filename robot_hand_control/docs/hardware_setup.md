# 硬件配置 (Hardware Setup)

> 对应 Prompt.md 背景与阶段0风险项。本文档记录当前已知硬件连接方式与通信参数。

## 1. 设备清单

| 设备 | 型号 | 数量 |
|------|------|------|
| 机械臂 | 睿尔曼 RM75-6F（7 自由度） | 1 |
| 灵巧手 | 灵心巧手 LinkerHand O6（**右手**，Modbus ID 0x27） | 1 |
| 上位机 | Linux Ubuntu 22.04 x86_64 | 1 |

## 2. 通信链路

### 2.1 RM75-6F（以太网 TCP/IP）

| 参数 | 值 |
|------|----|
| IP | 192.168.1.18 |
| 端口 | 8080 |
| 协议 | TCP/IP |
| SDK | RM_API2 v1.1.6（C++，`libapi_cpp.so`） |

### 2.2 O6（Modbus RTU，经 RM75 末端 RS485 透传）

**物理链路（当前唯一路径）**：

```
上位机 PC ──以太网(TCP/IP:8080)── RM75 控制器 ──末端 RS485── LinkerHand O6
```

O6 未直接连接上位机串口。所有 Modbus RTU 命令经 RM75 控制器透传。

| 参数 | 值 |
|------|----|
| 协议 | Modbus RTU |
| 波特率 | 115200（O6 固件固定，不可软件修改） |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | none |
| 从站地址 | 0x27（39，右手）；左手为 0x28 |
| 功能码 | 04（读输入寄存器）、16/0x10（写保持寄存器） |
| SDK | linkerhand-cpp-sdk v2.0.0（`linkerhand_cpp_sdk.so.2.0.0`） |

**RM75 透传配置要点**（源自 `rm_interface.h` ModbusConfig 组）：
- `rm_set_modbus_mode(handle, port=1, baudrate=115200, timeout)`：`port=1` 表示末端接口板 RS485 为 RTU 主站。
- 数据位 8 / 停止位 1 / 无校验为固定默认，不可配置（与 O6 需求一致）。
- `timeout` 单位百毫秒，对所有 Modbus 读写生效。
- Modbus RTU 模式与机械臂 RS485 控制模式互斥，关闭后恢复机械臂控制模式（460800 8N1）。
- 从站地址经 `rm_peripheral_read_write_params_t.device` 指定（=0x27）。

## 3. 连接时序

1. `rm_init(RM_TRIPLE_MODE_E)`。
2. `rm_create_robot_arm("192.168.1.18", 8080)` 连接 RM75。
3. `rm_set_modbus_mode(handle, 1, 115200, timeout)` 配置末端 RS485 透传。
4. 构造 `LinkerHandApi(O6, RIGHT, MODBUS)`。
5. 注入 TX/RX 回调（底层 = `RmPassthroughModbus`，内部调 RM 寄存器 API）。
6. 之后可读写 O6。

断开顺序相反：先断开 O6（回调释放），再 `rm_close_modbus_mode`，再 `rm_delete_robot_arm`。

## 4. 未确认项（硬件实测后补充）

| 项 | 状态 | 影响 |
|----|------|------|
| RM75 末端 RS485 引脚定义（A/B/地） | ⚠ 待确认 | 接线图见 `RM_API2/Demo/RMDemo_Cpp/RMDemo_ModbusRTU/End_Interface.png`、`End_IO_Interface_Diagram.png` |
| O6 压力传感器（华威科）数据格式 | ⚠ 待实测 | 寄存器 45-87，10×4 点阵，格式待验证 |
| 透传往返延迟 | ⚠ 待实测 | 影响采集频率与超时设置 |
| O6 上电时序 | ⚠ 待实测 | 是否需要先上电再连 RM 透传 |
| O6 是否随 RM75 末端供电 | ⚠ 待实测 | 供电方式待确认 |
