# O6 机械手 Modbus RTU 协议说明

> 来源文件：`O6机械手485协议简要说明.xlsx`
> 转换日期：2026-08-06

---

## 一、物理层与通信参数

| 参数 | 值 |
|------|----|
| 协议 | Modbus RTU |
| 默认波特率 | **115200**（暂不支持软件修改） |
| 数据位 | 8 |
| 停止位 | 1 |
| 校验位 | none |
| 支持功能码 | 04（读输入寄存器）、16（写保持寄存器） |
| 右手 Modbus ID | **0x27**（十进制 39） |
| 左手 Modbus ID | **0x28**（十进制 40） |

---

## 二、输入寄存器（功能码 04 — 只读）

### 2.1 关节位置（地址 0–5）

| 地址 | 名称 | 功能 | 范围 | 说明 |
|------|------|------|------|------|
| 0 | Current_Thumb_Pitch | 大拇指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 1 | Current_Thumb_Yaw | 大拇指横摆 | 0–255 | 小值向掌心靠拢，大值远离掌心 |
| 2 | Current_Index_Pitch | 食指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 3 | Current_Middle_Pitch | 中指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 4 | Current_Ring_Pitch | 无名指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 5 | Current_Little_Pitch | 小拇指弯曲 | 0–255 | 小值弯曲，大值伸直 |

### 2.2 关节转矩（地址 6–11）

| 地址 | 名称 | 功能 | 范围 |
|------|------|------|------|
| 6 | Current_Thumb_Pitch_Torque | 大拇指弯曲转矩 | 0–255 |
| 7 | Current_Thumb_Yaw_Torque | 大拇指横摆转矩 | 0–255 |
| 8 | Current_Index_Pitch_Torque | 食指弯曲转矩 | 0–255 |
| 9 | Current_Middle_Pitch_Torque | 中指弯曲转矩 | 0–255 |
| 10 | Current_Ring_Pitch_Torque | 无名指弯曲转矩 | 0–255 |
| 11 | Current_Little_Pitch_Torque | 小拇指弯曲转矩 | 0–255 |

### 2.3 关节速度（地址 12–17）

| 地址 | 名称 | 功能 | 范围 |
|------|------|------|------|
| 12 | Current_Thumb_Pitch_Speed | 大拇指弯曲速度 | 0–255 |
| 13 | Current_Thumb_Yaw_Speed | 大拇指横摆速度 | 0–255 |
| 14 | Current_Index_Pitch_Speed | 食指弯曲速度 | 0–255 |
| 15 | Current_Middle_Pitch_Speed | 中指弯曲速度 | 0–255 |
| 16 | Current_Ring_Pitch_Speed | 无名指弯曲速度 | 0–255 |
| 17 | Current_Little_Pitch_Speed | 小拇指弯曲速度 | 0–255 |

### 2.4 关节温度（地址 18–23）

| 地址 | 名称 | 功能 | 范围 |
|------|------|------|------|
| 18 | Current_Thumb_Pitch_Temperature | 大拇指弯曲温度 | 0–70 ℃ |
| 19 | Current_Thumb_Yaw_Temperature | 大拇指横摆温度 | 0–70 ℃ |
| 20 | Current_Index_Pitch_Temperature | 食指弯曲温度 | 0–70 ℃ |
| 21 | Current_Middle_Pitch_Temperature | 中指弯曲温度 | 0–70 ℃ |
| 22 | Current_Ring_Pitch_Temperature | 无名指弯曲温度 | 0–70 ℃ |
| 23 | Current_Little_Pitch_Temperature | 小拇指弯曲温度 | 0–70 ℃ |

### 2.5 关节错误码（地址 24–29）

| 地址 | 名称 | 功能 |
|------|------|------|
| 24 | Current_Thumb_Pitch_Error_Code | 大拇指错误码 |
| 25 | Current_Thumb_Yaw_Error_Code | 大拇指横摆错误码 |
| 26 | Current_Index_Pitch_Error_Code | 食指错误码 |
| 27 | Current_Middle_Pitch_Error_Code | 中指错误码 |
| 28 | Current_Ring_Pitch_Error_Code | 无名指错误码 |
| 29 | Current_Little_Pitch_Error_Code | 小拇指错误码 |

### 2.6 设备信息（地址 30–44）

| 地址 | 名称 | 功能 |
|------|------|------|
| 30 | Hand_freedom | 手自由度 |
| 31 | hand_version | 手版本 |
| 32–34 | hand_number (高位/中位/低位) | 手编码（3 寄存器拼接） |
| 35 | hand_direction | 左手/右手标识（ASCII `L` 或 `R`） |
| 36–38 | hardware_version (高位/中位/低位) | 硬件版本（3 寄存器拼接） |
| 39–41 | software_version (高位/中位/低位) | 软件版本（3 寄存器拼接） |
| 42–44 | mechanical_version (高位/中位/低位) | 机械版本（3 寄存器拼接） |

### 2.7 压力传感器（地址 45–47+N）

| 地址 | 名称 | 说明 |
|------|------|------|
| 45 | Pressure_Sensing_ID | 力传感器对应手指：1–5 对应大拇指→小拇指，0 表示无 |
| 46 | Pressure_Sensing_Specifications | 传感器规格：高 4 位 = 行数，低 4 位 = 列数（当前值 `0xA4` = 10 行 × 4 列） |
| 47 起 | Pressure_Sensing_Data_Start | 压力数据起始地址，长度由 `Pressure_Sensing_Specifications` 决定 |

> **读取长度**：传感器规格为 `0xA4`（十进制 164）时，结束地址 = 47 + 164 = **87**。
> 该段地址为华威科点阵式压力传感器数据，每个寄存器值范围 0–255。

---

## 三、保持寄存器（功能码 16 — 读写）

### 3.1 关节位置控制（地址 0–5）

| 地址 | 名称 | 功能 | 数值范围 | 说明 |
|------|------|------|----------|------|
| 0 | Thumb_Pitch | 大拇指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 1 | Thumb_Yaw | 大拇指横摆 | 0–255 | 小值向掌心靠拢，大值远离掌心 |
| 2 | Index_Pitch | 食指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 3 | Middle_Pitch | 中指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 4 | Ring_Pitch | 无名指弯曲 | 0–255 | 小值弯曲，大值伸直 |
| 5 | Little_Pitch | 小拇指弯曲 | 0–255 | 小值弯曲，大值伸直 |

### 3.2 关节转矩控制（地址 6–11）

| 地址 | 名称 | 功能 | 范围 |
|------|------|------|------|
| 6 | Thumb_Pitch_Torque | 大拇指弯曲转矩 | 0–255 |
| 7 | Thumb_Yaw_Torque | 大拇指横摆转矩 | 0–255 |
| 8 | Index_Pitch_Torque | 食指弯曲转矩 | 0–255 |
| 9 | Middle_Pitch_Torque | 中指弯曲转矩 | 0–255 |
| 10 | Ring_Pitch_Torque | 无名指弯曲转矩 | 0–255 |
| 11 | Little_Pitch_Torque | 小拇指弯曲转矩 | 0–255 |

### 3.3 关节速度控制（地址 12–17）

| 地址 | 名称 | 功能 | 范围 |
|------|------|------|------|
| 12 | Thumb_Pitch_Speed | 大拇指弯曲速度 | 0–255 |
| 13 | Thumb_Yaw_Speed | 大拇指横摆速度 | 0–255 |
| 14 | Index_Pitch_Speed | 食指弯曲速度 | 0–255 |
| 15 | Middle_Pitch_Speed | 中指弯曲速度 | 0–255 |
| 16 | Ring_Pitch_Speed | 无名指弯曲速度 | 0–255 |
| 17 | Little_Pitch_Speed | 小拇指弯曲速度 | 0–255 |

### 3.4 压力传感器数据选择（地址 18）

| 地址 | 名称 | 说明 | 范围 |
|------|------|------|------|
| 18 | Pressure_Sensing_Data_Selection | 选择读取的手指压力数据 | 0–5（1–5 对应大拇指→小拇指，0 无） |

### 3.5 波特率选择（地址 19）

| 地址 | 名称 | 说明 | 范围 |
|------|------|------|------|
| 19 | baud_rate_select | RS485 波特率切换 | 0=不变，1=115200，2=4000000 |

> **注意**：波特率切换仅在**每次上电后生效一次**。

---

## 四、O6 关节与手指映射

O6 灵巧手共 **6 个控制自由度**（6 通道）：

```
通道 0: Thumb_Pitch   — 大拇指弯曲
通道 1: Thumb_Yaw     — 大拇指横摆
通道 2: Index_Pitch   — 食指弯曲
通道 3: Middle_Pitch  — 中指弯曲
通道 4: Ring_Pitch    — 无名指弯曲
通道 5: Little_Pitch  — 小拇指弯曲
```

## 五、常用预设姿态

| 姿态 | 6 通道值 (T_Pitch, T_Yaw, I, M, R, L) |
|------|--------------------------------------|
| 完全张开 | `{255, 255, 255, 255, 255, 255}` |
| 握拳 | `{0, 0, 0, 0, 0, 0}` |
| 预抓取 | `{255, 128, 255, 255, 255, 255}` |

---

## 六、与 LinkerHand C++ SDK 的对应关系

此协议的寄存器读写已封装在 `linkerhand-cpp-sdk` 的 `LinkerHandApi` 中：

| Modbus 操作 | SDK 方法 |
|-------------|----------|
| 写寄存器 0–5（位置） | `LinkerHandApi::setPosition(pose)` |
| 读寄存器 0–5（位置） | `LinkerHandApi::getPosition()` |
| 写寄存器 12–17（速度） | `LinkerHandApi::setSpeed(speed)` |
| 读寄存器 12–17（速度） | `LinkerHandApi::getSpeed()` |
| 写寄存器 6–11（转矩） | `LinkerHandApi::setTorque(torque)` |
| 读寄存器 6–11（转矩） | `LinkerHandApi::getTorque()` |
| 读寄存器 18–23（温度） | `LinkerHandApi::getTemperature()` |
| 读寄存器 24–29（故障码） | `LinkerHandApi::getFaultCode()` |
| 读寄存器 31（版本） | `LinkerHandApi::getVersion()` |
| 读寄存器 45+（压力） | `LinkerHandApi::getForce()` |

---

## 七、Modbus 传输层实现要点

SDK 的回调模式要求调用方实现两个回调：

1. **`setModbusTxCallback`**：发送原始 Modbus 帧到串口
2. **`setModbusRxCallback`**：从串口接收并返回完整 Modbus 帧

这两个回调在 Adapter 中通过 `linkerhand-cpp-sdk` 的 `Modbus` 类实现，该类内部封装了 Linux `/dev/ttyUSB*` 或 RM75 末端 RS485 的串口操作。

详见 `linkerhand-cpp-sdk/include/communication/Modbus.h`。
