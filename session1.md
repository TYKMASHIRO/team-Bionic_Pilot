# session1.md — 任务一：拖动示教 → 记录轨迹 → 复现

> 本文件是**任务一（阶段一）专用工作 Prompt**。下次开始任务一前，先读本文件，再决定下一步动作。
> 它保存了已完成的 SDK 审计结果、架构决策、工程状态与进度，避免重复劳动。
> 主文档：`Prompt.md`（完整工程规范）；本文件是任务一的执行聚焦版。

---

## 1. 任务一目标

通过**拖动示教**让操作者手动拖着机械臂运动，程序**记录轨迹**，然后**复现**。

核心是两件事：
1. **机械臂（RM75-6F）与灵巧手（O6）的通信**：O6 不直连电脑，而是经 RM75 末端 RS485 用 Modbus RTU 透传访问。
2. **时间戳与协同**：两设备要用**统一时间轴**同步记录、同步复现，不能各自 sleep 凑合。

任务一的主线闭环：
```
拖动示教（arm 记录轨迹）＋ O6 同步状态记录
   → 生成统一时间轴轨迹（arm + hand）
   → 校验 → 按阶段复现（先 arm 到位 → hand 手型 → 协同动作）
```

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`dev/cyb`（个人同步分支）
- 已提交：`ee41ccb`（阶段2 RM75 适配器完成）

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成（docs/ 5 份文档已提交） |
| 阶段1 工程骨架 | ✅ 完成（编译/测试/CLI 全部通过） |
| 阶段2 RM75 Adapter | ✅ 完成（编译验证 + 单测 + 硬件测试注册，`ee41ccb`） |
| 阶段3-7 | ⏳ 未开始（下一步：阶段3 O6 Adapter） |

**阶段2 完成时修复的编译问题**（下次不必重踩）：
- `RealManAdapter.hpp` 里的 `typedef struct rm_robot_handle rm_robot_handle;` 与 `rm_define.h` 的匿名结构体 typedef **冲突** → 已删除前置声明（PIMPL 下头文件本就不需要厂商类型）
- `rm_to_error()` 返回 `Error` 但调用方需要 `Result` → 新增 `rm_to_result()`（rc==0→ok，否则→fail），15 处调用已替换；`connect()` 里 `Result::fail(rm_to_error(...))` 用法保持不变
- `RmErrorMap.cpp` 的 `using namespace robotics::domain;` 只在 `rm_to_error` 函数体内生效，`rm_to_result` 内需写全 `domain::Result`
- 硬件 preset 曾配置失败：`set_tests_properties` 前缺 `add_test(NAME hw_arm_connect ...)`，且硬件测试没链接 `${GTEST_LIB}` → 已补，现在注册进 ctest 并默认 DISABLED

**阶段2 验收**：`mock`(7)/`debug`(8)/`hardware`(8+1 DISABLED) 三个 preset 构建+测试全部通过。

---

## 3. 构建命令（每次开工先跑）

```bash
cd robot_hand_control
# Mock 模式（无硬件，最快回归）
cmake --preset mock && cmake --build build/mock -j$(nproc) && ctest --test-dir build/mock
# Debug（含 realman/linkerhand 驱动）
cmake --preset debug && cmake --build build/debug -j$(nproc)
# Release / ASan（可选）
cmake --preset release && cmake --build build/release
cmake --preset asan && cmake --build build/asan
```

CLI（Mock 模式，进程隔离，每次命令独立连接）：
```bash
build/mock/apps/robotctl doctor
build/mock/apps/robotctl arm connect | status | home --dry-run | stop
build/mock/apps/robotctl arm drag-teach start --record | stop
build/mock/apps/robotctl hand connect | status | preset open --dry-run | preset close --dry-run
# 真实运动需显式：  ... --enable-motion
```

---

## 4. 已核实的本地 SDK 事实（不要重新审计，直接使用）

> 以下全部来自本地头文件/示例，2026-08-07 核实。详细出处见 `robot_hand_control/docs/vendor_api_mapping.md`。

### 4.1 RM75-6F（睿尔曼 API2 v1.1.6，`RM_API2/C++/include/`）

**连接/生命周期**
```c
rm_init(RM_TRIPLE_MODE_E);                                  // 三线程（UDP推模式必需）
rm_robot_handle* h = rm_create_robot_arm("192.168.1.18", 8080);  // id>0 成功, -1 失败
rm_delete_robot_arm(h);  rm_destroy();
rm_api_version();       // "1.1.6"
```

**状态拉取**（7 自由度 `ARM_DOF=7`，关节角单位 °，位置 m，欧拉角 rad）
```c
rm_get_arm_all_state(h, &rm_arm_all_state_t);       // 电流/使能/温度/电压/关节错误码
rm_get_current_arm_state(h, &rm_current_arm_state_t); // pose + joint[7] + err[24]
rm_get_force_data(h, &rm_force_data_t);             // 六维力 force_data[6]
```

**运动**（`v`=速度0-100，`block`=0非阻塞/1多线程阻塞）
```c
rm_movej(h, float joint[7], v, r, traj_connect, block);   // 关节运动
rm_movej_p(h, rm_pose_t, v, r, traj_connect, block);      // 位姿运动(关节空间)
rm_movel(h, rm_pose_t, v, r, traj_connect, block);        // 直线
rm_set_arm_slow_stop(h);   // 缓停
rm_set_arm_stop(h);        // 急停，轨迹不可恢复
```

**拖动示教 + 轨迹复现（★ 任务一核心）**
```c
rm_start_drag_teach(h, trajectory_record);   // 1=记录轨迹
rm_stop_drag_teach(h);
rm_drag_trajectory_origin(h, block);         // 20% 速度回轨迹起点（复现前必调）
rm_run_drag_trajectory(h, block);            // 开始复现
rm_pause_drag_trajectory / rm_continue_drag_trajectory / rm_stop_drag_trajectory
rm_save_trajectory(h, "file.txt", &num);     // 保存到文件
```

**Modbus RTU 透传（★ O6 通信关键）**——注意是**高层寄存器 API，非原始字节透传**
```c
rm_set_modbus_mode(h, port, baudrate, timeout);
// port: 0=控制器RS485, 1=末端接口板RS485(用于O6), 2=控制器从站
// baudrate: 115200（O6 固定）; timeout 单位百毫秒; 数据位8/停止1/无校验固定
rm_close_modbus_mode(h, port);

typedef struct { int port; int address; int device; int num; } rm_peripheral_read_write_params_t;
// device = Modbus 从站地址（O6 右手 0x27）; port=1

rm_read_input_registers(h, params, int* data);            // 0x04, 单次1个
rm_read_multiple_input_registers(h, params, int* data);   // 0x04, 2<num<13（3~12个）
rm_write_single_register(h, params, int data);            // 0x06
rm_write_registers(h, params, int* data);                 // 0x10, num≤10
```

**RM 返回码约定**（头文件无错误码枚举，逐函数文档）：0成功 / 1参数错误 / -1发送失败 / -2接收超时 / -3解析失败 / -4到位校验或四代不支持 / -5单线程超时 / -6停止规划 / -7三代不支持。已封装在 `RmErrorMap.cpp`。

### 4.2 LinkerHand O6（SDK v2.0.0，`linkerhand-cpp-sdk/include/`）

**构造与回调注入模式（O6 SDK 不持有 socket，回调即传输层）**
```cpp
LinkerHandApi hand(O6, RIGHT, MODBUS);   // O6=枚举, RIGHT=0x27
hand.setModbusTxCallback(ModbusTxCallback);  // 发完整 Modbus RTU 帧（含CRC）
hand.setModbusRxCallback(ModbusRxCallback);  // 收完整响应帧

// Tx: int32_t(uint8_t slave_id, uint16_t reg_addr, const uint8_t* data, uintptr_t data_len)
// Rx: int32_t(uint8_t slave_id, uint16_t* reg_addr_out, uint8_t* data_out, uint8_t* data_len_out)
// 返回 0 成功 / -1 失败; Rx 需校验 data_out[0] == slave_id
```

**控制与状态**（6 通道，位置 raw 0-255，小值弯曲大值伸直；失败返回空 vector）
```cpp
hand.setPosition({uint8_t x6});   // {255×6}=张开 {0×6}=握拳 {255,128,255,255,255,255}=预抓取
hand.setSpeed({x6});  hand.setTorque({x6});
hand.getPosition() / getSpeed() / getTorque() / getTemperature() / getFaultCode();
hand.getVersion() → std::string;
hand.getForce() → vector<vector<vector<uint8_t>>>（压力，O6=4×10）
// 没有 open()/close()/stop()/setPreset(); 单通道控制需自行切片 vector
```

**传输层抽象**（`IModbus.h`）——RmPassthroughModbus 要实现这个接口
```cpp
class IModbus {
    virtual bool isOpen() const = 0;
    virtual void close() = 0;
    virtual bool sendRawFrame(const uint8_t* data, size_t len) = 0;      // 不做CRC
    virtual int receiveCompleteFrame(uint8_t* buf, size_t max, int timeout_ms=500) = 0;
    virtual int transact(const uint8_t* req, size_t req_len, uint8_t* resp, size_t max, int timeout_ms=500) = 0;
};
// 直连串口实现是 Modbus("/dev/ttyUSB0", 115200) —— 本项目不用这条路
```

### 4.3 ★ 核心架构矛盾（任务一最大难点，务必理解）

- O6 SDK 回调要求收发**完整 Modbus RTU 帧**（含从站地址/功能码/寄存器地址/数据/CRC）。
- RM75 透传是**高层寄存器 API**（`rm_read_multiple_input_registers` 等），不暴露原始字节。
- **方案**：`RmPassthroughModbus : IModbus` 在内部做三层翻译：
  1. **解析** O6 SDK 发来的完整帧（slave_id/功能码/地址/数据/CRC）；
  2. **映射**到 RM 高层 API（0x04→读输入、0x10→写保持等）；
  3. **重组** RM 返回的寄存器值为完整 Modbus RTU 响应帧（含 CRC）回给 O6 SDK。
- 这样 `LinkerHandApi` 上层无感知，无需改 SDK。骨架文件已在 `drivers/linkerhand/transport/RmPassthroughModbus.{hpp,cpp}`（阶段3实现）。

---

## 5. 架构决策（已定，不要推翻）

1. **分层单向依赖**：Application → Task → Skill → Domain 接口 → Vendor Adapters → SDK。禁止反向/跨层。
2. **厂商隔离**：`rm_*_t`、`LinkerHandApi`、Modbus 寄存器结构**只允许出现在 `drivers/` 内**，上层只见 `IRobotArm`/`IDexterousHand` + 统一领域类型（`RobotArmState`/`DexterousHandState`/`Pose`/`Error` 等）。
3. **两设备模块互不依赖**：机械臂模块不得 include 灵巧手模块，反之亦然；协同只能由上层 Skill/Task（如 `combined.synchronized_replay`）完成。
4. **时间戳**：`steady_clock` 用于控制时序/新鲜度，`system_clock` 用于日志/文件名；两者不混用。`CombinedRobotState` 携带双设备 `time_delta_ns` 与 `sync_quality`。
5. **安全**：真实运动默认关闭；所有运动命令须 `--enable-motion` 显式授权 + SafetySupervisor 检查（新鲜度/限位/力/温度）。急停后需人工复位。
6. **命令串行化**：同一设备运动命令经 `CommandScheduler` 单线程执行，支持取消/超时。
7. **轨迹数据不硬编码在 C++**：轨迹点放文件（CSV/YAML），C++ 只负责逻辑。
8. **线程**：采集线程只做「获取→转换→入队」，不写文件不执行 Skill；写盘用后台线程；所有线程受控停止。详见 `docs/threading_model.md`。

工程目录（`robot_hand_control/`）：
```
apps/robotctl         CLI
include/robotics/     领域类型 + 抽象接口
src/                  服务实现（services/safety/trajectory/orchestration/infrastructure）
drivers/realman       RM75 适配（阶段2）
drivers/linkerhand    O6 适配 + transport/RmPassthroughModbus（阶段3）
drivers/mock          Mock 设备（连接/运动/拖动示教/故障注入，可无硬件测试）
config/               YAML 配置示例（robot/safety/logging）
data/                 recordings/ trajectories/ skills/ calibration/
docs/                 architecture / vendor_api_mapping / implementation_plan / hardware_setup / threading_model
tests/                unit/ integration/ hardware/ fixtures/
tools/                config_validator（可运行），trajectory_inspector / recording_converter（占位）
```

---

## 6. 任务一执行路线（下一步建议）

| 步骤 | 内容 | 验收 |
|------|------|------|
| 2️⃣ 收尾 | 编译验证 RealManAdapter（debug preset）+ RmErrorMap 单测 | ✅ 完成（`ee41ccb`，三 preset 全过） |
| 3️⃣ O6 Adapter | 实现 `RmPassthroughModbus`（帧解析→RM映射→响应重组）+ `LinkerHandAdapter` | Mock 传输层单测帧解析/CRC |
| 4️⃣ 状态同步记录 | RM75/O6 采集线程 + StateStore + Recorder 后台写盘 + 事件标记 | Mock 双设备连续记录，时间戳单调 |
| 5️⃣ 轨迹复现 | 轨迹格式/加载/校验（单调/NaN/范围/起点偏差）/调速/Dry-run/Mock 复现 | Mock 复现通过前禁止真实组合运动 |
| 6️⃣ Skill | 先做 `arm.drag_teach_record`、`combined.synchronized_replay`、`combined.safe_release` | 双设备协同复现闭环 |

**注意**：阶段5 验收明确要求「Mock 复现完全通过前，不允许真实组合运动」。真实设备联调放最后。

---

## 7. 硬性约束（违反即返工）

- 不得编造厂商 API；使用前先查本地头文件（`RM_API2/C++/include/`、`linkerhand-cpp-sdk/include/`）。
- 每个阶段结束必须可编译、可测试。
- 真实运动默认禁止；未配置的安全项默认拒绝运动。
- 厂商类型/错误码/句柄不得泄漏到上层业务模块。
- 轨迹点不得硬编码在 C++ 源文件中。
- 日志写盘/轨迹保存不得阻塞设备状态采集线程。
- 不允许在厂商 SDK 回调线程中执行耗时业务逻辑。
- 资料缺失时建 TODO + Unsupported 返回值，不得伪造实现伪装成功。

---

## 8. 相关文档索引

| 文档 | 用途 |
|------|------|
| `Prompt.md` | 完整工程规范（总纲，任务一只取相关章节） |
| `session2.md` | 阶段2（RM75 Adapter）执行聚焦版：Domain↔RM 映射、编译坑清单、阶段3 入口 |
| `robot_hand_control/docs/vendor_api_mapping.md` | 厂商 API 逐项映射（含待确认项） |
| `robot_hand_control/docs/architecture.md` | 分层架构 |
| `robot_hand_control/docs/implementation_plan.md` | 阶段0-7 计划 |
| `robot_hand_control/docs/hardware_setup.md` | 硬件链路/参数 |
| `robot_hand_control/docs/threading_model.md` | 线程模型 |
| `robot_hand_control/docs/../docs/O6_ModbusRTU_Protocol.md` | O6 寄存器完整映射 |
