# session5.md — 阶段五：轨迹管理与复现

> 本文件是**任务一（阶段五）专用工作 Prompt**。下次涉及阶段5、或做阶段6（Skill 框架，尤其 `combined.synchronized_replay`）前，先读本文件，再决定下一步动作。
> 它保存了阶段5 的**交付状态、轨迹格式决策、编译坑清单、验收基线与阶段6 入口**，避免重复劳动与重踩。
> 主文档：`Prompt.md`；阶段4 执行版：`session4.md`；本文件是阶段5 的执行聚焦版。

---

## 1. 阶段5 目标

把录制数据变成**可校验、可调速、可复现**的轨迹对象，实现双设备（RM75 + O6）按统一时间轴同步回放：

1. **轨迹格式**：统一时间轴（`t_offset_ns` 相对首帧单调）+ 双设备状态 + 事件标记。
2. **加载**：复用 states.csv 110 列契约读回为 `Trajectory`（不新造格式）。
3. **校验**：格式/版本/关节数/时间戳单调/NaN/范围。
4. **调速**：时间轴缩放倍率（`t_offset_ns / speed`）。
5. **Dry-run**：校验不运动（起点偏差/设备在线/安全许可前置全查）。
6. **Mock 复现**：逐点下发 arm.move_joint + hand.set_joint_positions；支持取消/超时/执行报告。

阶段5 只做**轨迹管理**，不做 Skill 框架（阶段6）与座舱 Skill（阶段7）。

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`master`（阶段5 工作分支）
- 阶段4 完成于 `dev/cyb`；阶段5 在 master 上推进（见 git status）。

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成（docs/ 文档） |
| 阶段1 工程骨架 | ✅ 完成 |
| 阶段2 RM75 Adapter | ✅ 完成 |
| 阶段3 O6 Adapter | ✅ 完成 |
| 阶段4 统一状态与同步记录 | ✅ 完成（session4.md） |
| 阶段5 轨迹管理与复现 | ✅ **完成**（代码 + Mock 测试，本次） |
| 阶段6 Skill 框架 | 待开始 |

---

## 3. 构建命令（每次开工先跑）

```bash
cd robot_hand_control
# Mock 模式（无硬件，两驱动 OFF）
cmake --preset mock && cmake --build build/mock -j$(nproc) && ctest --test-dir build/mock
# Debug（含 realman/linkerhand 驱动；真实设备用此构建）
cmake --preset debug && cmake --build build/debug -j$(nproc) && ctest --test-dir build/debug
# ASan（地址/UB 检测）
cmake --preset asan && cmake --build build/asan -j$(nproc) && ctest --test-dir build/asan
```

**阶段5 验收基线**：`mock`(17) / `debug`(19) / `asan`(19) 全部通过（新增 4 个轨迹测试目标：`unit_trajectory_csv_loader` / `unit_trajectory_validator` / `unit_trajectory_replayer` / `integration_trajectory_replay`）。

**Mock 手工验收链路**：
```bash
# 1) 录制 1s
./build/mock/apps/robotctl record data/recordings --duration 1 --rate 50
# 2) 检查录制能否加载为轨迹（+校验）
./build/mock/tools/trajectory_inspector data/recordings/<session> --validate
# 3) 导入 → 列 → 检查 → 校验
./build/mock/apps/robotctl trajectory import data/recordings/<session>
./build/mock/apps/robotctl trajectory list
./build/mock/apps/robotctl trajectory inspect traj_<session>
./build/mock/apps/robotctl trajectory validate traj_<session>
# 4) dry-run（不运动）→ 真实复现（Mock，需 --enable-motion）
./build/mock/apps/robotctl trajectory replay traj_<session> --dry-run
./build/mock/apps/robotctl trajectory replay traj_<session> --enable-motion --speed 5
```

**真实设备 CLI 用法**（阶段6 硬件验证复用）：
```bash
# 先 --real 录一段（用户确认每步真实动作）
./build/debug/apps/robotctl --real record data/recordings --duration 10 --rate 5
# 导入 + dry-run 校验（不运动）
./build/debug/apps/robotctl trajectory import data/recordings/<session>
./build/debug/apps/robotctl trajectory replay traj_<session> --dry-run
# 真实复现（必须 --enable-motion，设备需已装好）
./build/debug/apps/robotctl --real trajectory replay traj_<session> --enable-motion --speed 1
```

---

## 4. 阶段5 架构决策（已定，不要推翻）

1. **轨迹格式 = 录制契约复用**：`Trajectory` 的 states 部分直接复用 `states.csv` 110 列（`CsvStateColumns` 单一来源）。表头/数据行同源生成，录制写出的列序与轨迹读回的列序永远一致。**不新造第二套列定义。**
2. **统一时间轴**：`TrajectoryPoint.t_offset_ns` 相对首帧单调偏移（首帧=0）。加载时 `t_offset_ns = steady_ns - 首帧_steady_ns`；与录制墙钟解耦。
3. **调速语义 = 时间轴缩放倍率**：目标时刻 `t0 + t_offset_ns / speed`。`speed=2` 两倍速、`0.5` 半速、`<=0` 兜底 1.0。**不做插值/重采样**，仅缩放点间等待。
4. **Dry-run 校验完整**：校验 → 设备在线 → 起点偏差（报告）→ 返回（**不进入安全评估/运动**）。Prompt.md 要求 dry-run 检查起点偏差/设备在线/安全许可，其中"安全许可"在真实路径由 `ApplicationService` 的 `--enable-motion` 门控前置完成（见 §6 决策 2）。
5. **厂商隔离延续**：`TrajectoryReplayer` 只依赖 `IRobotArm/IDexterousHand/ISafetySupervisor/IClock`，头文件无厂商类型。
6. **磁盘资产复用 loader 布局**：`<data_dir>/<id>/` 下 `states.csv + events.csv + metadata.txt + manifest.txt`；`TrajectoryCsvLoader` 直接读资产目录（复用全部解析逻辑），`manifest.txt` 存轨迹专属字段（id/name/source/created），加载时 manifest 覆盖 loader 推断字段。
7. **import 幂等**：`trajectory_id = "traj_" + session_id`；同一录制目录重复导入覆盖（save 同名 id）。
8. **复现顺序（红线）**：校验 → 设备在线 → 起点偏差 → [dry-run 返回] → 安全评估 → 起点对齐 → 播放。任一前置失败即停止并给出 `failed_stage`。

---

## 5. 阶段5 交付清单

| 组件 | 位置 | 说明 |
|------|------|------|
| `CsvStateColumns` | `src/domain/recording/CsvStateColumns.{hpp,cpp}` | 110 列契约单一来源：names/write_row/parse_row + `csv_escape_shared`/`csv_split` |
| `CsvRecordSink`（重构） | `src/infrastructure/recording/CsvRecordSink.cpp` | 表头/数据行改由 `CsvStateColumns::write_row` 驱动（行为不变，已过原测试） |
| `TrajectoryCsvLoader` | `src/trajectory/TrajectoryCsvLoader.{hpp,cpp}` | 录制/资产目录 → `Trajectory`（states+events+metadata 校验） |
| `TrajectoryValidator` | `src/trajectory/TrajectoryValidator.{hpp,cpp}` | 校验报告 + 范围限制（arm ±3.5 rad / hand 0-255） |
| `TrajectoryReplayer` | `src/trajectory/TrajectoryReplayer.{hpp,cpp}` | 复现执行器（调速/取消/超时/起点对齐/执行报告） |
| `TrajectoryRepository`（扩展） | `src/trajectory/TrajectoryRepository.{hpp,cpp}` | 磁盘持久化 + `import_recording` + 跨进程可见 |
| `ApplicationService`（扩展） | `src/orchestration/ApplicationService.cpp` | trajectory_import/list/load/validate/replay + repo 注入 |
| CLI | `apps/robotctl/main.cpp` | `trajectory import/list/inspect/validate/replay` |
| 工具 | `tools/trajectory_inspector/main.cpp` | 录制/资产目录加载 + `--validate` |
| 测试 | `tests/unit/trajectory_*.cpp` + `tests/integration/trajectory_replay_integration_test.cpp` | 4 个新测试目标 |
| 文档 | `docs/trajectory_format.md` | 轨迹格式规范 |

---

## 6. 编译坑清单（已修复，重踩即浪费时间）

1. **CsvStateColumns 缺 `<iomanip>`**：`std::setprecision/std::defaultfloat` 编译错 → 加 include。
2. **`csv_escape_shared` 未暴露**：CsvRecordSink 的 write_event/write_command 需转义函数但当时在匿名命名空间 → 提到 `CsvStateColumns.hpp/.cpp` 公开。
3. **fail_load 签名**：`fail_load(const char*)` 无法拼接 std::string → 改为 `Result fail_load(const std::string&)`。
4. **TrajectoryCsvLoader 缺 `<algorithm>`**：`std::sort` 报错 → 加 include。
5. **SafetySupervisor::evaluate 签名**：需要 `CombinedRobotState`（单参数），不是 `(arm_state, hand_state)` → 构造 combined 再调。
6. **replayer 未用变量**：`last_arm_valid/found_last_arm` 无用 → 删除（避免 -Wall）。
7. **测试里 ASSERT_TRUE 用在非 void 函数**：`write_recording` 内部 `ASSERT_TRUE(write_row(...))` 非法（宏返回 void）→ 改直接调用。
8. **集成测试残留资产**：`::testing::TempDir()` 固定 `/tmp/`，同名 `traj_repo` 跨测试运行残留导致 `list()` 扫到旧资产 → 测试前 `fs::remove_all`。
9. **replayer 测试事件单位**：events.csv 的 steady_ns 偏移写成 15ms 而时间轴是 ns → 改成 ns 级偏移。
10. **dry-run 需设备在线**：TrajectoryReplayer 的 dry-run 分支在"设备在线"检查之后（Prompt.md 语义：dry-run 校验设备在线/起点偏差）→ 测试先 connect 再 dry-run。

---

## 7. 阶段5 执行回顾（验收方式）

**Mock 手工链路（已实测通过，见 §3 命令）**：
- `trajectory_inspector data/recordings/<session> --validate` → 解析 51 点、时长 ~1.0s、校验通过。
- `import` → `traj_<session>`；`list`/`inspect` 显示 src/pts/dur/ver/calib + 首尾点。
- `replay --dry-run` → `成功 stage=dry_run points=0/51`（不运动）。
- `replay` 无 `--enable-motion` → 拒绝 `[safety] 真实运动未启用`（退出码 1）。
- `replay --enable-motion --speed 5` → `成功 stage=replay points=51/51`，`final=arm[ok j=(...)] hand[...]`（Mock 立即到位，末点一致）。

**测试覆盖（17/17 通过）**：
- `unit_trajectory_csv_loader`：加载/事件关联/版本不匹配/缺文件/空/坏行/仓库往返。
- `unit_trajectory_validator`：通过/空/版本/非单调/NaN/越界/hand 越界/缺设备警告。
- `unit_trajectory_replayer`：dry-run/无效轨迹/未连接+safety 门控/末点到位/取消/起点对齐/超时/align 关闭拒绝。
- `integration_trajectory_replay`：record→import→list→load→validate→dry-run→真实复现（含未启用运动拒绝）；跨仓库实例加载（进程隔离）。

---

## 8. 未验证项（阶段6 或后续实测）

- **真实组合运动未执行**：按 Prompt.md 红线，Mock 复现完全通过前不允许真实组合运动；本次 Mock 已 17/17 通过，阶段6 可用 `--real trajectory replay` 做首次真机验证。
- **起点对齐的真实运动代价**：偏差超阈值时先 movej 首点再播放，真机（尤其 O6 依赖 RM 供电）下需确认对齐时 hand 无下发的行为符合预期。
- **取消/超时的真实语义**：Mock 立即到位，无法体现真实 move_joint 阻塞时长对"下发耗时超间隔不补眠"的影响；真机复现需实测跟随性。
- **events.csv 真机事件关联**：事件挂在"第一个 t_offset >= 偏移"的点，若两事件时间戳相同会 `;` 连接；真机录制速率低时事件分辨率需确认。

---

## 9. 硬性约束（违反即返工）

1. **不得编造厂商 API**；轨迹只复用已实现的录制/加载能力。
2. **每阶段可编译可测试**：mock/debug/asan 全过才收尾。
3. **真实运动默认禁止**，必须 `--enable-motion` 显式许可（`SafetySupervisor` + CLI 双门控）。
4. **厂商类型/句柄不泄漏**：`void*` handle + PIMPL；`rm_*_t/LinkerHandApi` 只在 drivers/ 内。
5. **轨迹点不得硬编码在 C++ 源文件**（测试用例除外）。
6. **不允许在厂商 SDK 回调线程执行耗时业务逻辑**。
7. **Mock 复现完全通过前，不允许真实组合运动**（阶段5 验收红线）。
8. **RM 多寄存器读写 `data` 是 2*num 个 int8 原始字节**（session4 §5.1，阶段6 触 RM 多读多写沿用）。
9. **连接 arm→hand、断开 hand→arm**（O6 由 RM75 末端供电）。
10. **O6 压力读取保持跳过**（session4 §5.2 用户决策；启用需拆事务 + 确认硬件）。

---

## 10. 下一步：阶段6（Skill 框架）

阶段6 首个 Skill 即依赖阶段5：

- `combined.synchronized_replay`：直接委托 `ApplicationService::trajectory_replay`（参数：trajectory_id、speed、dry_run、timeout）。
- `arm.drag_teach_record`：拖动示教 + 同步记录双设备（复用阶段4 Recorder + 阶段5 导入）。
- 每个 Skill：manifest + 参数 + 前置条件 + 状态机 + 结果 + 测试（Prompt.md §13 起）。

阶段6 开工前：重读 `Prompt.md` Skill 章节 + `session4.md` §5（RM 契约）+ 本文件 §3（构建基线）与 §6（编译坑）。

---

## 11. 相关文档索引

- `Prompt.md`：主文档（阶段5 定义 §1200-1212；Skill 定义 §13 起）。
- `docs/trajectory_format.md`：轨迹格式规范（本阶段产出）。
- `docs/implementation_plan.md`：阶段5 ✅ 完成记录 + 阶段6 入口。
- `session4.md`：阶段4 执行版（RM 契约/压力跳过/运动链路验证）。
