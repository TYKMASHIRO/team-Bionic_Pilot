# session6.md — 阶段六：Skill 框架

> 本文件是**任务一（阶段六）专用工作 Prompt**。下次涉及阶段6、或做阶段7（座舱 Skill，尤其复用 Skill 框架 / Runtime / manifest）前，先读本文件，再决定下一步动作。
> 它保存了阶段6 的**交付状态、架构决策、编译坑清单、验收基线与阶段7 入口**，避免重复劳动与重踩。
> 主文档：`Prompt.md`；阶段5 执行版：`session5.md`；框架规范：`docs/skill_framework.md`。

---

## 1. 阶段6 目标

把动作封装为**可复用、可编排、可取消、可安全门控**的 Skill：

1. **Skill 接口**：`ISkill`（descriptor / validate / execute）+ 标准化描述（Prompt.md §599 字段）。
2. **Manifest**：YAML 文件承载参数/前置条件/状态机/超时/取消/恢复/安全/技能专属数据。
3. **运行时**：资源互斥 → validate → dry-run 短路 → 执行 → 结果装配（final_state/时长/安全标志）。
4. **真实运动双重门控**：`--enable-motion` + SafetySupervisor + SkillBase 安全门。
5. **7 个基础 Skill**（Prompt.md §1216）：arm ×2 / hand ×3 / combined ×2。
6. **测试**：unit（manifest / runtime / basic）+ integration（combined 全链路）。

阶段6 只做 Skill 框架，不做座舱 Skill（阶段7）。

---

## 2. 当前工程状态（截至 2026-08-07）

- 工程根：`team-Bionic_Pilot/robot_hand_control/`
- 当前分支：`master`
- 阶段5 完成于 `44614ac`（dev/cyb）；阶段6 在本分支推进。

| 阶段 | 状态 |
|------|------|
| 阶段0 SDK 审计 | ✅ 完成 |
| 阶段1 工程骨架 | ✅ 完成 |
| 阶段2 RM75 Adapter | ✅ 完成 |
| 阶段3 O6 Adapter | ✅ 完成 |
| 阶段4 统一状态与同步记录 | ✅ 完成（session4.md） |
| 阶段5 轨迹管理与复现 | ✅ 完成（session5.md） |
| 阶段6 Skill 框架 | ✅ **完成**（代码 + Mock 测试，本次） |
| 阶段7 座舱动作 Skill | 待开始 |

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

**阶段6 验收基线**：`mock`(21) / `debug`(23) / `asan`(23) 全部通过（阶段5 之后新增 4 个测试目标：`unit_skill_manifest` / `unit_skill_runtime` / `unit_skill_basic` / `integration_skill_combined`）。

**Mock 手工验收链路**（已实测）：
```bash
./build/mock/apps/robotctl --skills-dir ./skills skill list
# → 7 个 Skill 全部注册（id@version + real-motion 标记 + 描述）

# dry-run：只校验不运动
./build/mock/apps/robotctl --skills-dir ./skills skill hand.open '{}' --dry-run
# → success=true stage=dry_run

# 真实运动被安全门拒绝（未 --enable-motion）
./build/mock/apps/robotctl --skills-dir ./skills skill hand.open '{}'
# → success=false state=safety_stopped stage=safety（真实运动未启用）

# 显式许可后真实执行
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion skill hand.open '{}'
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion skill hand.close '{}'
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion skill arm.move_to_safe_pose '{}'
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion skill combined.safe_release '{}'

# 拖动示教 + 记录 + 导入轨迹
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion \
    skill arm.drag_teach_record '{"out_dir":"/tmp/drag","duration_s":0.3,"import":true}'
# → success=true ... ; traj=traj_rec_<session>

# 同步复现刚录制的轨迹
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion \
    skill combined.synchronized_replay '{"trajectory_id":"traj_rec_<session>","timeout_ms":5000}'
# → success=true

# 非法参数拒绝
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion \
    skill hand.apply_preset '{"preset":"bogus"}'
# → success=false stage=validate（非法预设名）
./build/mock/apps/robotctl --skills-dir ./skills --enable-motion \
    skill combined.synchronized_replay '{"trajectory_id":"does_not_exist"}'
# → success=false stage=validate（轨迹不存在）
```

---

## 4. 阶段6 架构决策（已定，不要推翻）

1. **Manifest = 文件 YAML**（`skills/*.yaml`，yaml-cpp 加载）。`id`/`version`/`required_resources` 必填；其余缺省取默认；技能专属数据（`safe_pose`/`safe_pose_speed`/`preset`）放 manifest，**禁止硬编码 C++**。
2. **真实运动双重门控**：`--enable-motion`（CLI/ApplicationService）→ `SafetySupervisor::real_motion_enabled()` → `SkillBase::require_real_motion()`。任一未过 → `safety_stopped`，不运动。
3. **超时并入 cancel 回调**：`desc.timeout > 0` 时，`SkillRuntime` 把 deadline 并入 `exec.cancel`；Skill 在阶段边界轮询 `cancelled(params)` 受控停止。超时/取消**最终语义由 Skill 判定**（SynchronizedReplay 依据 ReplayReport 映射 Cancelled/TimedOut）。
4. **资源互斥在 SkillRuntime**（非 dry-run）：`required_resources` 展开（`combined`→arm+hand）+ **去重**，同一设备同一时刻只有一个 Skill 持有执行权。
5. **dry-run 校验完整但短路**：仍走 `validate()`（参数/预置条件/轨迹加载校验），但**不占资源、不执行**，`failed_stage=dry_run` 成功返回。
6. **SkillContext 依赖注入**：arm/hand/safety/clock/store/trajectory_repo + `RecordingHooks`；Skill 不直接依赖具体 sink/文件格式/厂商类型。
7. **`combined.safe_release` = 手张开 + 臂回安全位**（用户确认，session5 决策延续）；开始后即完成两阶段（`finish_current_command`），阶段边界不查取消。
8. **状态机阶段 = manifest `execution_stages` + C++ `failed_stage` 统一命名**；失败结果必须带 `failed_stage` + 明确错误（Prompt.md §548/§1268）。
9. **工厂隔离**：`SkillFactory` 由 manifest id 构造具体实例；未知 id 返回 nullptr + error，不崩溃。

---

## 5. 阶段6 交付清单

| 组件 | 位置 | 说明 |
|------|------|------|
| `ISkill` + `SkillDescriptor`/`SkillParameter`/`SkillParams`/`SkillResult` | `include/robotics/interfaces/ISkill.hpp`、`include/robotics/domain/results/SkillResult.hpp` | 接口 + 标准化描述 + 执行参数 + 结果 |
| `SkillManifest` | `include/robotics/skills/SkillManifest.hpp` + `src/skills/SkillManifest.cpp` | YAML manifest → descriptor（必填校验 + 技能专属数据） |
| `SkillParams` | `include/robotics/skills/SkillParams.hpp` + `src/skills/SkillParams.cpp` | 参数解析（bool/double/int/string/double_list）+ hand preset 解析 |
| `SkillContext` | `include/robotics/skills/SkillContext.hpp` | 依赖注入 + `RecordingHooks` |
| `SkillBase` | `include/robotics/skills/SkillBase.hpp` + `src/skills/SkillBase.cpp` | 公共基类：预置条件→dry-run→安全门→run；辅助函数 |
| `SkillRegistry` | `include/robotics/skills/SkillRegistry.hpp` + `src/skills/SkillRegistry.cpp` | 线程安全 id→ISkill |
| `SkillRuntime` | `include/robotics/skills/SkillRuntime.hpp` + `src/skills/SkillRuntime.cpp` | 统一执行入口（资源→validate→dry-run→execute→结果装配） |
| `SkillFactory` | `include/robotics/skills/SkillFactory.hpp` + `src/skills/SkillFactory.cpp` | id→具体 Skill 实例 |
| 7 个 Skill | `src/skills/ArmMoveToSafePoseSkill.*`、`ArmDragTeachRecordSkill.*`、`HandPresetSkill.*`、`CombinedSynchronizedReplaySkill.*`、`CombinedSafeReleaseSkill.*` | 见 §6 |
| 7 个 manifest | `skills/*.yaml` | 参数/前置条件/状态机/超时/取消/恢复/安全/专属数据 |
| ApplicationService 接入 | `src/orchestration/ApplicationService.cpp` | `load_skills`/`skill_list`/`run_skill` + skill_ctx_ 装配 + 资源仲裁注入 |
| CLI | `apps/robotctl/main.cpp` | `--skills-dir` + `skill list` / `skill <id> [params] [--dry-run]` |
| 测试 | `tests/unit/skill_manifest_test.cpp`、`skill_runtime_test.cpp`、`skill_basic_test.cpp`、`tests/integration/skill_combined_integration_test.cpp` | 4 个新测试目标 |
| 文档 | `docs/skill_framework.md` | Skill 框架规范 |

---

## 6. 7 个基础 Skill 明细

| id | required_resources | 参数 | 状态机阶段 | 关键行为 |
|----|--------------------|------|------------|----------|
| `arm.move_to_safe_pose` | `arm` | `joints`(可选覆盖 manifest)、`speed` | validate → move_joint | 以 `safe_pose_speed` 移到安全位 |
| `arm.drag_teach_record` | `combined` | `out_dir`(必填)、`duration_s`(0=直到取消)、`rate_hz`(默认50)、`import`(默认true) | validate → start_drag_teach → start_recording → wait_drag → stop_recording → stop_drag_teach → import_recording | 示教+同步记录双设备；取消=受控停止；导入后摘要附 `traj=<id>` |
| `hand.open` | `hand` | `preset`(可选) | validate → apply_preset(Open) | {255×6} |
| `hand.close` | `hand` | `preset`(可选) | validate → apply_preset(Close) | {0×6} |
| `hand.apply_preset` | `hand` | `preset`(必填 open/close/pregrasp/custom) | validate → apply_preset | 参数覆盖 manifest |
| `combined.synchronized_replay` | `arm`,`hand` | `trajectory_id`(必填)、`speed`、`arm_speed_ratio`、`align_start`、`timeout_ms` | validate(加载+校验轨迹) → replay | 委托 `TrajectoryReplayer`；ReplayReport→SkillResult |
| `combined.safe_release` | `arm`,`hand` | `joints`(可选)、manifest `safe_pose` | validate → hand.open → arm 安全位 | 开始即完成（finish_current_command） |

---

## 7. 编译坑清单（已修复，重踩即浪费时间）

1. **`const` 丢弃限定符**：`desc()` 非 const 访问器在 const 成员函数里调用报错 → 改 `descriptor()`（ISkill 的 const 访问器）。
2. **`ReplayOptions` 未声明**：CombinedSynchronizedReplaySkill.cpp 缺 `#include "robotics/trajectory/TrajectoryReplayer.hpp"` → 补 include。
3. **资源重复占用**：`arm.drag_teach_record` manifest 曾写 `["arm","combined"]` 导致 arm 被占两次 → manifest 改 `["combined"]` **并且** SkillRuntime 加设备展开去重。
4. **SkillRuntime 花括号失衡**：去重编辑遗留多余 `}`，导致 `'params' does not name a type` / `'skill' was not declared`（mock 构建曾用旧二进制掩盖）→ 移除多余右花括号后重编译，**新二进制重跑全部 smoke test**。
5. **`infra` 命名空间**：测试里 `std::make_shared<infra::SystemClock>()` 报 `'infra' was not declared` → 加 `using namespace robotics;`。
6. **测试本地 `kArmDof/kHandDof` 与 domain 常量冲突** → 删除本地常量，用 `robotics::domain` 的。
7. **`set_real_motion_enabled` 返回 void**：`ASSERT_TRUE(safety->set_real_motion_enabled(true))` 非法（宏需要 bool）→ 直接调用。
8. **Mock 手 connect 默认张开 255**：安全门测试断言"手仍在 0"错误 → 改为比较 execute 前后状态不变。
9. **manifest 测试缺技能专属数据**：`combined.safe_release`/`arm.move_to_safe_pose` 测试 manifest 未写 `safe_pose` → `resolve_safe_pose` 校验失败（"安全位需要 7 个关节值，实际 0"）→ 测试 manifest 补 `safe_pose`。
10. **JSON 字符串缺右括号**：`R"({"trajectory_id":")" + id + R"(","timeout_ms":8000)"` 少了结尾 `}` → yaml-cpp 解析报 `end of map flow not found`，skill 报"缺少 trajectory_id" → 补 `}`。

---

## 8. 阶段6 执行回顾（验收方式）

**测试覆盖（21/21 mock 通过）**：
- `unit_skill_manifest`（7 用例）：全字段加载（含参数/阶段/超时/取消/恢复/safety/safe_pose/preset）、缺 id / 缺 version / 空资源 / 坏 YAML / 缺省值 / 文件不存在。
- `unit_skill_runtime`（8 用例）：dry-run 短路（validate 调用但不 execute、不占资源）、validate 失败前置、成功执行 + 资源释放、资源冲突（acquire 失败）、combined 展开去重、取消映射（Cancelled）、超时映射（并入 cancel）、失败映射（Failed）。
- `unit_skill_basic`（9 用例）：hand.open→255 / hand.close→0 / apply_preset 参数覆盖 / 非法预设拒绝 / move_to_safe_pose（manifest 与 joints 参数两种来源）/ 安全门拒绝（手不动）/ safe_release（手张开）/ drag_teach 缺 out_dir 校验 + 快速执行。
- `integration_skill_combined`（2 用例）：load_skills 注册 7 个 → 录制 → 导入 → synchronized_replay → safe_release → 未注册 id 查找失败；dry-run 不运动。

**Mock 手工链路（已实测通过，见 §3）**：
- `skill list` → 7 个全部注册。
- `hand.open` dry-run → `stage=dry_run`；无 `--enable-motion` → `safety_stopped`；有 → success。
- `hand.close` / `arm.move_to_safe_pose` / `combined.safe_release` → success。
- `hand.apply_preset` `preset=close` 参数覆盖 → success；`preset=bogus` → validate 拒绝。
- `arm.drag_teach_record`（out_dir/duration_s=0.3/import）→ success + `traj=traj_rec_<session>`。
- `combined.synchronized_replay`（trajectory_id=刚导入的 traj）→ success；不存在的 id → validate 拒绝。

---

## 9. 未验证项（阶段7 或后续实测）

- **真实硬件 Skill 执行**：本次 Mock 全过；阶段7 或验收时用 `--real skill ...` 做首次真机验证（每个真实动作需用户确认，见 §10 真机链路）。
- **拖动示教真实语义**：Mock 的 `start_drag_teach` 立即返回；真机需确认 RM 拖动示教期间事件/状态采集节奏、示教结束自动停录。
- **超时/取消真实语义**：Mock 立即到位，无法体现真实 move_joint 阻塞时长；真机复现需实测跟随性（session5 §8 延续）。
- **`combined.safe_release` 真实姿势**：manifest `safe_pose` 目前用工程默认 {0.1..0.7}；真机须按机械臂实际安全位标定后更新 `skills/arm.move_to_safe_pose.yaml` 与 `combined.safe_release.yaml`。
- **资源仲裁并发测试**：单线程测试覆盖；并发（多线程同发两个 Skill）未覆盖，阶段7 若引入并发编排需补测。

---

## 10. 真实设备 Skill 链路（阶段7/验收复用）

```bash
cd robot_hand_control
# 0) 真机 SKILL 验证前：连接 arm→hand（O6 由 RM 末端供电），先单设备各自验证过
# 1) 列表
./build/debug/apps/robotctl --real --skills-dir ./skills skill list
# 2) 安全位（必须 --enable-motion；用户确认安全区域内动作）
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion skill arm.move_to_safe_pose '{}'
# 3) 手开/关（O6 依赖 RM 供电，先保证 arm 已连）
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion skill hand.open '{}'
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion skill hand.close '{}'
# 4) 拖动示教（人手拖臂，同时记录；duration_s>0 到点自动停）
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion \
    skill arm.drag_teach_record '{"out_dir":"data/recordings","duration_s":10,"import":true}'
# 5) 同步复现（必须 --enable-motion）
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion \
    skill combined.synchronized_replay '{"trajectory_id":"traj_<session>","speed":1}'
# 6) 安全释放（手张开 + 臂安全位）
./build/debug/apps/robotctl --real --skills-dir ./skills --enable-motion skill combined.safe_release '{}'
```

> 真实运动红线：每个真实动作前用户确认；`arm.move_to_safe_pose` 与 `combined.safe_release` 的安全位**必须先按真机标定**（见 §9）。

---

## 11. 硬性约束（违反即返工）

1. **不得编造厂商 API**；Skill 只复用已实现的 Domain 接口与轨迹能力。
2. **每阶段可编译可测试**：mock/debug/asan 全过才收尾。
3. **真实运动默认禁止**，必须 `--enable-motion` 显式许可（CLI + SafetySupervisor + SkillBase 三重门控）。
4. **厂商类型/句柄不泄漏**：Skill 头文件无厂商类型；`rm_*_t/LinkerHandApi` 只在 drivers/ 内。
5. **技能专属数据（safe_pose/preset 等）禁止硬编码在 C++**，一律走 manifest。
6. **轨迹点不得硬编码在 C++ 源文件**（测试用例除外）。
7. **Mock 复现完全通过前，不允许真实组合运动**（阶段5 验收红线延续）。
8. **RM 多寄存器读写 `data` 是 2*num 个 int8 原始字节**（session4 §5.1）。
9. **连接 arm→hand、断开 hand→arm**（O6 由 RM75 末端供电）。
10. **O6 压力读取保持跳过**（session4 §5.2 用户决策）。

---

## 12. 下一步：阶段7（座舱动作 Skill）

Skill 框架已稳定，可逐座舱 Skill 实现：

- `cockpit.approach_control_stick` / `grasp_control_stick` / `release_control_stick`
- `cockpit.push_throttle` / `press_button` / `rotate_knob`

每个：manifest（新技能专属数据按需加字段）+ 参数 + 前置条件 + 状态机 + 结果 + 测试；
**不得写成无状态 API 调用串**（Prompt.md §1245-1246），必须用 Skill 阶段/成功判据/超时/安全策略/恢复策略。

阶段7 开工前：重读 `Prompt.md` 阶段7 章节 + `session4.md` §5（RM 契约）+ 本文件 §3（构建基线）与 §7（编译坑）+ `docs/skill_framework.md`（框架规范）。

---

## 13. 相关文档索引

- `Prompt.md`：主文档（Skill 定义 §13、动作分级 §561-597、阶段6 §1214、阶段7 §1234）。
- `docs/skill_framework.md`：Skill 框架规范（本阶段产出）。
- `docs/implementation_plan.md`：阶段6 ✅ 完成记录 + 阶段7 入口。
- `session5.md`：阶段5 执行版（轨迹管理，synchronized_replay 依赖）。
- `session4.md`：阶段4 执行版（RM 契约/压力跳过/运动链路验证）。
- `docs/trajectory_format.md`：轨迹格式规范。
