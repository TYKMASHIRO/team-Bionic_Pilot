# 厂商 API 映射 (Vendor API Mapping)

> 本文件将项目抽象接口与本地厂商 SDK 的实际 API 一一对应。
> 所有签名均来自**本地 SDK 头文件**（审计于 2026-08-07），不得凭记忆编造。

## 1. RM75-6F 机械臂（睿尔曼 API2 v1.1.6）

- 头文件：`RM_API2/C++/include/{rm_interface.h, rm_define.h, rm_service.h, rm_version.h}`
- 库：`RM_API2/C++/linux/linux_x86_c++_v1.1.6/libapi_cpp.so`（debug: `linux_x86_c++_debug_v1.1.6/libapi_cpp_debug.so`）
- SDK 版本宏：`SDK_VERSION = "1.1.6"`（`rm_version.h:9`）

### 1.1 连接/生命周期

| 语义 | 厂商 API | 说明 |
|------|----------|------|
| 初始化 SDK | `int rm_init(rm_thread_mode_e mode)` | `RM_TRIPLE_MODE_E=2` 三线程（推模式 UDP 必需） |
| 连接机械臂 | `rm_robot_handle* rm_create_robot_arm(const char* ip, int port)` | 返回句柄 id>0 成功，id=-1 失败 |
| 断开 | `int rm_delete_robot_arm(rm_robot_handle* handle)` | - |
| 销毁全局 | `int rm_destroy(void)` | 释放所有连接/线程 |
| API 版本 | `char* rm_api_version(void)` | 返回版本字符串 |

`rm_robot_handle` 定义（`rm_define.h:954-956`）：`typedef struct { int id; } rm_robot_handle;`

### 1.2 状态获取

| 语义 | 厂商 API | 数据结构 |
|------|----------|----------|
| 当前状态（拉） | `int rm_get_current_arm_state(handle, rm_current_arm_state_t* state)` | `rm_current_arm_state_t` = pose + `float joint[ARM_DOF]` + err |
| 全状态（拉） | `int rm_get_arm_all_state(handle, rm_arm_all_state_t* state)` | 电流/使能/温度/电压/错误码数组 |
| 关节角度（拉） | `int rm_get_joint_degree(handle, float* joint, int* speed)` | - |
| 六维力 | `int rm_get_force_data(handle, rm_force_data_t* data)` | `rm_force_data_t` 含 4 组 `float[6]` |
| UDP 主动上报（推） | `rm_set_realtime_push(handle, rm_realtime_push_config_t config)` + `rm_realtime_arm_state_call_back(cb)` | 三线程模式必需；回调收 `rm_realtime_arm_joint_state_t` |
| 事件回调（到位等） | `rm_get_arm_event_call_back(rm_event_callback_ptr cb)` | 收 `rm_event_push_data_t` |

**关键结构体（`rm_define.h`）**：

```c
// 关节状态 rm_joint_status_t (269-278)
float joint_current[ARM_DOF];   // mA，精度 0.001mA
bool  joint_en_flag[ARM_DOF];
uint16_t joint_err_code[ARM_DOF];
float joint_position[ARM_DOF];  // °，精度 0.001°
float joint_temperature[ARM_DOF];
float joint_speed[ARM_DOF];     // °/s，精度 0.01
// ARM_DOF = 7 (rm_define.h:12)

// 位姿 rm_pose_t (171-176)
typedef struct {
    rm_position_t position;   // m
    rm_quat_t     quaternion; // w,x,y,z
    rm_euler_t    euler;      // rx,ry,rz rad
} rm_pose_t;

// 六维力 rm_force_data_t (501-506)
float force_data[6];            // N / N·m
float zero_force_data[6];
float work_zero_force_data[6];
float tool_zero_force_data[6];

// 错误 rm_err_t (248-252)
uint8_t err_len; int err[24];
```

### 1.3 运动控制

| 语义 | 厂商 API |
|------|----------|
| 关节运动 | `int rm_movej(handle, const float* joint, int v, int r, int trajectory_connect, int block)` |
| 位姿运动（关节空间） | `int rm_movej_p(handle, rm_pose_t pose, int v, int r, int trajectory_connect, int block)` |
| 直线运动 | `int rm_movel(handle, rm_pose_t pose, int v, int r, int trajectory_connect, int block)` |
| 缓停 | `int rm_set_arm_slow_stop(handle)` |
| 急停 | `int rm_set_arm_stop(handle)`（轨迹不可恢复） |
| 暂停/继续 | `rm_set_arm_pause(handle)` / `rm_set_arm_continue(handle)` |

- `block`：`RM_MOVE_NBLOCK=0` 非阻塞；`RM_MOVE_MULTI_BLOCK=1` 多线程阻塞；`RM_MOVE_SINGLE_BLOCK(timeout)` 单线程阻塞。
- `trajectory_connect`：`RM_TRAJECTORY_DISCONNECT_E=0` / `RM_TRAJECTORY_CONNECT_E`。
- `v` = 速度（关节运动 0-100），`r` = 半径/加速度，单位语义见厂商文档。

### 1.4 拖动示教与轨迹复现（★ 任务一核心）

| 语义 | 厂商 API | 说明 |
|------|----------|------|
| 开始拖动示教 | `int rm_start_drag_teach(handle, int trajectory_record)` | `trajectory_record=1` 记录轨迹 |
| 结束拖动示教 | `int rm_stop_drag_teach(handle)` | - |
| 复合拖动示教 | `int rm_start_multi_drag_teach(handle, int mode, int singular_wall)` | 三代控制器 |
| 复合拖动（新参数） | `int rm_start_multi_drag_teach_new(handle, rm_multi_drag_teach_t)` | `rm_multi_drag_teach_t` = free_axes[6]+frame+singular_wall |
| 设置拖动灵敏度 | `int rm_set_drag_teach_sensitivity(handle, int grade)` | 0~100 |
| 运动到轨迹起点 | `int rm_drag_trajectory_origin(handle, int block)` | 20% 速度 |
| 轨迹复现开始 | `int rm_run_drag_trajectory(handle, int block)` | 须先回起点 |
| 暂停/继续/停止复现 | `rm_pause_drag_trajectory` / `rm_continue_drag_trajectory` / `rm_stop_drag_trajectory` | - |
| 保存轨迹 | `int rm_save_trajectory(handle, const char* name, int* num)` | 保存文件路径+点数 |

### 1.5 坐标系

| 语义 | 厂商 API |
|------|----------|
| 设置工具坐标系 | `int rm_set_manual_tool_frame(handle, rm_frame_t frame)` |
| 切换工具坐标系 | `int rm_change_tool_frame(handle, const char* tool_name)` |
| 获取当前工具系 | `int rm_get_current_tool_frame(handle, rm_frame_t* frame)` |
| 设置工作坐标系 | `int rm_set_manual_work_frame(handle, const char* work_name, rm_pose_t pose)` |
| 切换工作坐标系 | `int rm_change_work_frame(handle, const char* work_name)` |
| 获取当前工作系 | `int rm_get_current_work_frame(handle, rm_frame_t* frame)` |

### 1.6 Modbus RTU 透传（★ O6 通信关键）

| 语义 | 厂商 API |
|------|----------|
| 配置 Modbus RTU 模式 | `int rm_set_modbus_mode(handle, int port, int baudrate, int timeout)` |
| 关闭 Modbus 模式 | `int rm_close_modbus_mode(handle, int port)` |
| 读输入寄存器(0x04) | `int rm_read_input_registers(handle, rm_peripheral_read_write_params_t, int* data)` — 单次 1 个 |
| 读多个输入寄存器 | `int rm_read_multiple_input_registers(handle, params, int* data)` — 2<num<13 |
| 读保持寄存器(0x03) | `int rm_read_holding_registers(handle, params, int* data)` — 单次 1 个 |
| 读多个保持寄存器 | `int rm_read_multiple_holding_registers(handle, params, int* data)` — 2<num<13 |
| 写单个寄存器(0x06) | `int rm_write_single_register(handle, params, int data)` |
| 写多个寄存器(0x10) | `int rm_write_registers(handle, params, int* data)` — num≤10 |

参数结构体 `rm_peripheral_read_write_params_t`（`rm_define.h:520-528`）：
```c
typedef struct {
    int port;    // 0-控制器RS485, 1-末端接口板RS485, 3-控制器ModbusTCP
    int address; // 数据起始地址
    int device;  // 外设设备地址（= Modbus 从站地址）
    int num;     // 数量
} rm_peripheral_read_write_params_t;
```

**O6 经 RM 透传的调用约定**：
- `port = 1`（末端接口板 RS485）
- `device = 0x27`（39，右手）
- `baudrate = 115200`（O6 固定，不可改）
- `timeout` 单位百毫秒
- O6 支持功能码：04（读输入寄存器）、16/0x10（写保持寄存器）

**⚠ 待确认项**：RM75 是否支持向 O6 发起原始字节透传存在功能码白名单限制（0x04/0x03/0x06/0x10 均已覆盖 O6 协议所需；O6 用 0x04 读、0x10 写）。`rm_write_registers` num≤10 覆盖 O6 一次写 6 通道位置。O6 压力数据（寄存器 45-87，43 个）需分段读，每段 ≤12 个。

### 1.7 其它

| 语义 | 厂商 API |
|------|----------|
| 清除系统错误 | `int rm_clear_system_err(handle)` |
| 读取软件信息 | `int rm_get_arm_software_info(handle, rm_arm_software_version_t*)` |
| 六维力拖动模式 | `rm_set_force_drag_mode(handle, int mode)` |
| 力位混合 | `rm_set_force_position(handle, ...)` |

### 1.8 RM 错误码约定

RM API2 返回 `int`，**头文件无错误码枚举**，逐函数 `@return` 文档约定：

| 值 | 含义 |
|----|------|
| 0 | 成功 |
| 1 | 控制器返回 false（参数错误或状态错误） |
| -1 | 数据发送失败 / 未找到句柄 |
| -2 | 数据接收失败 / 控制器超时（六维力可能表示非六维力版本） |
| -3 | 返回值解析失败 |
| -4 | 因函数而异（到位校验失败/四代不支持） |
| -5 | 单线程模式超时 |
| -6 | 机械臂停止运动规划 |
| -7 | 三代控制器不支持 |

## 2. LinkerHand O6 灵巧手（SDK v2.0.0）

- 头文件：`linkerhand-cpp-sdk/include/{api/LinkerHandApi.h, core/Common.h, communication/*.h}`
- 库：`linkerhand-cpp-sdk/lib/linux/x86_64/linkerhand_cpp_sdk.so.2.0.0`（SONAME `linkerhand_cpp_sdk.so.2`）

### 2.1 枚举（`core/Common.h`）

```cpp
enum LINKER_HAND { L6, L7, L10, L20, L21, L25, O6, G20, O20 };   // O6
enum HAND_TYPE { LEFT = 0x28, RIGHT = 0x27 };                    // 本项目用 RIGHT
enum COMM_TYPE { CAN, MODBUS, ETHERCAT };
```

### 2.2 LinkerHandApi（`api/LinkerHandApi.h`）

**构造与回调**：

```cpp
LinkerHandApi(const LINKER_HAND& handJoint, const HAND_TYPE& handType,
              const COMM_TYPE commType = COMM_TYPE::CAN);
// 本项目：LinkerHandApi(O6, RIGHT, MODBUS)

void setModbusTxCallback(ModbusTxCallback);  // 发送完整 Modbus RTU 帧
void setModbusRxCallback(ModbusRxCallback);  // 接收完整 Modbus RTU 响应帧
void freeModbusCallback();
```

回调类型（`communication/CommunicationCallbacks.h`）：
```cpp
using ModbusTxCallback = std::function<int32_t(
    uint8_t slave_id, uint16_t reg_addr, const uint8_t* data, uintptr_t data_len)>;
using ModbusRxCallback = std::function<int32_t(
    uint8_t slave_id, uint16_t* reg_addr_out, uint8_t* data_out, uint8_t* data_len_out)>;
// 返回 0 成功，-1 失败
```

**位置/速度/转矩控制（O6 = 6 通道，raw 0-255）**：

```cpp
void setPosition(const std::vector<uint8_t>& pose);   // 0-255, 6 元素；小值弯曲、大值伸直
void setPositionArc(const std::vector<double>& pose); // 弧度版本
std::vector<uint8_t> getPosition();
void setSpeed(const std::vector<uint8_t>& speed);
std::vector<uint8_t> getSpeed();
void setTorque(const std::vector<uint8_t>& torque);
std::vector<uint8_t> getTorque();
```

**状态读取**：

```cpp
std::vector<std::vector<std::vector<uint8_t>>> getForce();  // 5指×行×列（O6=4×10）
std::string getVersion();
std::vector<uint8_t> getTemperature();
std::vector<uint8_t> getFaultCode();
void clearFaultCode(...);  // 仅 L25/L20 支持
void setEnable(...); void setDisable(...);  // 仅 L25/L20
```

**重要语义**：
- **没有** `open()`/`close()`/`stop()`/`setPreset()`/单通道 set 方法。张开=setPosition(255×6)，握拳=setPosition(0×6)，预抓取=`{255,128,255,255,255,255}`。单通道控制需自行切片 vector。
- get 系列失败时返回**空 vector**（不是异常）。
- SDK 不持有 socket，回调即传输层。回调可能从 SDK 内部线程调用，回调体必须线程安全。
- `clearFaultCode/setEnable/setDisable` 注释仅支持 L25/L20——O6 不支持，Adapter 需返回 Unsupported 或忽略。

### 2.3 Modbus 传输层接口（`communication/IModbus.h`）

```cpp
class IModbus {
    virtual bool isOpen() const = 0;
    virtual void close() = 0;
    virtual bool sendRawFrame(const uint8_t* data, size_t length) = 0; // 不做 CRC
    virtual int receiveCompleteFrame(uint8_t* buffer, size_t max_size, int timeout_ms = 500) = 0;
    virtual int transact(const uint8_t* request, size_t request_len,
                         uint8_t* response, size_t max_response_len,
                         int timeout_ms = 500) = 0;  // 线程安全原子事务
};
```

- SDK 内部已把寄存器操作封装成完整 Modbus RTU 帧（含 CRC）交给 TX 回调。
- `Modbus` 类（`communication/Modbus.h`）是直连串口实现：`Modbus("/dev/ttyUSB0", 115200)`，构造后 `initialize()`/`isOpen()`。
- **本项目路径无 /dev/ttyUSB0**，而是经 RM75 末端 RS485 透传。已实现 `RmPassthroughModbus : IModbus`（阶段3），底层调用 RM 的寄存器 API。
- `DirectSerialModbus : IModbus` 已建接口 + 占位（阶段3），未实现直连串口，不伪造未验证 API。

**RmPassthroughModbus 关键事实（阶段3 已实现，`drivers/linkerhand/transport/`）**：
- 头文件以 `void*` 接收 RM 句柄（避免 `struct rm_robot_handle` 前置声明与 `rm_define.h` 匿名 typedef 冲突，见 session2 §6.1）。
- **SDK 回调是 Tx/Rx 分离调用**（示例 `test_o6_modbus_0.cpp`）：Tx → `sendRawFrame`（内部执行 RM 事务并缓存响应帧），Rx → `receiveCompleteFrame`（取缓存）。
- 功能码映射：0x04 → `rm_read_input_registers`(单)/`rm_read_multiple_input_registers`(多)；0x03 → 保持寄存器版本；0x06 → `rm_write_single_register`；0x10 → `rm_write_registers`（num≤10，RM 不返回响应帧，故按 0x10 标准响应回显 addr+count）。
- **0x10 写后不读回校验**：O6 只支持 0x04 读，且位置目标写入后当前值滞后，回读校验不可靠。
- 单次事务用 `std::mutex` 串行化；`ModbusFrameCodec` 提供 CRC16/帧解析/组帧（纯函数，mock 可测）。

### 2.4 O6 寄存器映射（`docs/O6_ModbusRTU_Protocol.md`）

| 寄存器类型 | 地址 | 内容 |
|-----------|------|------|
| 输入(04) 0-5 | 位置 | Thumb_Pitch/Yaw, Index, Middle, Ring, Little |
| 输入(04) 6-11 | 转矩 | 同上 6 通道 |
| 输入(04) 12-17 | 速度 | 同上 |
| 输入(04) 18-23 | 温度 | 同上 |
| 输入(04) 24-29 | 故障码 | 同上 |
| 输入(04) 30-44 | 设备信息 | 自由度/版本/编号/左右手/硬件软件机械版本 |
| 输入(04) 45-87 | 压力 | 华威科点阵传感器 |
| 保持(0x10) 0-5 | 位置目标 | 0-255 |
| 保持(0x10) 6-11 | 转矩目标 | 0-255 |
| 保持(0x10) 12-17 | 速度目标 | 0-255 |
| 保持(0x10) 18 | 压力选择 | 0-5 |
| 保持(0x10) 19 | 波特率选择 | 0/1/2 |

### 2.5 待确认/风险项

**阶段3（O6 Adapter）已确认项**：

| 项 | 状态 | 说明 |
|----|------|------|
| SDK 回调为 Tx/Rx 分离 | ✅ 已从源码确认 | 示例 `test_o6_modbus_0.cpp`：Tx→`sendRawFrame`，Rx→`receiveCompleteFrame`；`RmPassthroughModbus` 按此设计 |
| 0x10 写后无 RM 响应帧 | ✅ 已从源码确认 | RM 寄存器写 API 不返回 Modbus 帧，`RmPassthroughModbus` 按 0x10 标准响应回显 addr+count |
| RM 寄存器读写数量限制 | ✅ 已从源码确认 | 单读 1、多读 3~12、多写 ≤10；O6 6 通道位置可用多读一次取 6 个 |
| `setEnable/setDisable` | ✅ O6 不支持 | `clearError` 返回 Unsupported；`get_state` 用 0x04 读，不涉及 |
| `rm_robot_handle` 前置声明冲突 | ✅ 已规避 | header 用 `void*`，`.cpp` 内 cast（见 session2 §6.1） |

**仍需硬件实测的项（阶段3 无法在无硬件环境验证）**：

| 项 | 状态 | 说明 |
|----|------|------|
| RM75 透传时序/波特率 | ⚠ 待硬件 | `rm_set_modbus_mode(handle, 1, 115200, timeout)` 实机是否能稳定收发 O6 帧；timeout 取值（当前默认 500ms→RM 侧 5 单位）需实测调优 |
| O6 实际响应帧 | ⚠ 待硬件 | `parse_read_response` 假设标准 Modbus 04 响应（addr/FC/bytecount/data/CRC），实机需比对 SDK 期望帧 |
| 压力数据格式 | ⚠ 待硬件 | 华威科 10×4 点阵，寄存器 45-87；SDK `getForce()` 返回三维数组，映射关系待硬件验证（阶段3 仅留接口，未实现） |
| `rm_write_registers` 透传行为 | ⚠ 待硬件 | 是否要求严格 8N1 帧间隔、O6 是否丢弃过快连续帧 |
| 单通道控制 | ⚠ 待验证 | 无单通道 set 方法，需切片 vector；行为与并发写依赖实测 |

## 3. 抽象接口 → 厂商 API 对照总表

| 抽象接口 | RM75 实现（RealManAdapter） | O6 实现（LinkerHandAdapter，阶段3） |
|----------|----------------------------|------------------------------|
| connect | `rm_init` + `rm_create_robot_arm` | `rm_set_modbus_mode(1,115200)` + 构造 `LinkerHandApi(O6,RIGHT,MODBUS)` + 回调注入（Tx→`sendRawFrame`，Rx→`receiveCompleteFrame`） |
| disconnect | `rm_delete_robot_arm` | 析构 + `freeModbusCallback` |
| getState | `rm_get_arm_all_state` + `rm_get_force_data` | `getPosition/getSpeed/getTorque/getTemperature/getFaultCode`（经 0x04 读，`valid=false` 时返回空） |
| moveJoint | `rm_movej` | `setPosition`（经 0x10 写保持寄存器 0-5） |
| stop | `rm_set_arm_slow_stop` | 重新发送当前 6 通道位置（0x10 写当前值保持） |
| emergencyStop | `rm_set_arm_stop` | 同 stop（手无独立急停） |
| startDragTeach | `rm_start_drag_teach(handle, 1)` | N/A（示教由手部人工操作） |
| stopDragTeach | `rm_stop_drag_teach` | N/A |
| replayTrajectory | `rm_drag_trajectory_origin` + `rm_run_drag_trajectory` | 按轨迹点 `setPosition`（由上层 Skill 同步） |
| setToolFrame | `rm_set_manual_tool_frame` | N/A |
| setWorkFrame | `rm_set_manual_work_frame` | N/A |
| getForceTorque | `rm_get_force_data` | `getForce`（压力，不同语义；阶段3 未实现，留接口） |
| clearError | `rm_clear_system_err` | `clearFaultCode`（O6 不支持 → 返回 Unsupported） |
| healthCheck | `rm_get_arm_software_info` + 状态 | `getVersion` + 状态 |
