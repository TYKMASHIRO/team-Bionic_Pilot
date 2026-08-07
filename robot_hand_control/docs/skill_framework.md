# Skill 框架规范（阶段6）

> 主文档：`Prompt.md`（Skill 定义 §13 起、阶段6 §1214）；执行版：`session6.md`。
> 本文件描述 Skill 框架的**契约**：manifest 格式、接口、运行时流程、资源仲裁、7 个基础 Skill、CLI 用法、验收基线。

---

## 1. 分层定位

```
Application / Agent / GUI / CLI
        │
        ▼
   Task（上层编排，阶段7）
        │
        ▼
   Skill 层（本阶段）── ISkill + SkillRuntime + SkillRegistry + YAML manifest
        │
        ▼
   Domain 接口（IRobotArm / IDexterousHand / ISafetySupervisor / IClock /
              IStateStore / ITrajectoryRepository / ResourceManager）
        │
        ▼
   Vendor Adapter + Mock（drivers/）
```

红线（Prompt.md 二、八）：**Skill 内不得出现串口 / Socket / SDK 句柄 / 寄存器地址 / 厂商类型**；
设备互斥只在 Skill 层（`SkillRuntime` + `ResourceManager`），不落在设备内部。

---

## 2. Skill 接口（`include/robotics/interfaces/ISkill.hpp`）

```cpp
class ISkill {
public:
    virtual ~ISkill() = default;
    virtual const SkillDescriptor& descriptor() const = 0;   // manifest 元数据
    virtual Result validate(const SkillParams& params) = 0;  // 只校验，不运动
    virtual SkillResult execute(const SkillParams& params) = 0; // 完整状态机（同步阻塞）
};
```

- `validate()`：预置条件 + 参数合法性。**不运动、不占资源**。dry-run 也走这里（校验完整）。
- `execute()`：执行状态机。真实运动前由 `SkillBase` 统一过安全门（`real_motion` 门控）。

### SkillDescriptor（manifest 数据 + 技能专属数据）

| 字段 | 来源 | 说明 |
|------|------|------|
| `id` / `name` / `version` / `description` | manifest 必填（name 缺省=id） | 唯一标识 |
| `required_resources` | manifest 必填（非空） | `arm` / `hand` / `combined`（展开为 arm+hand，运行时去重） |
| `parameters` | manifest | 参数表（name/type/required/default/description） |
| `preconditions` | manifest | 人类可读预置条件；C++ 在 `check_preconditions` 强制 |
| `execution_stages` / `success_conditions` / `failure_conditions` | manifest | 状态机阶段 / 成功判据 / 失败判据 |
| `timeout` | manifest `timeout_ms` | 超时并入取消回调（Skill 自行判断超时语义） |
| `cancellation_policy` / `recovery_policy` | manifest | 取消 / 恢复策略元数据 |
| `real_motion` | manifest `safety_profile.real_motion` | 是否真实运动（需 `--enable-motion` 权限） |
| `trajectory_references` / `calibration_references` | manifest | 轨迹 / 标定引用 |
| `safe_pose` / `safe_pose_speed` / `preset` | manifest | 技能专属数据（禁止硬编码在 C++） |

### SkillParams（执行参数）

```cpp
struct SkillParams {
    std::string parameters_json;   // 参数（JSON/YAML 对象；yaml-cpp 解析，YAML 是 JSON 超集）
    bool dry_run = false;          // 只校验不运动
    std::function<bool()> cancel;  // 返回 true = 请求取消（超时并入此回调）
};
```

### SkillResult（执行结果，至少包含 Prompt.md §548 字段）

`skill_id` / `skill_version` / `success` / `error` / `failed_stage` / `final_state` /
`duration_ms` / `trajectory_version` / `config_version` / `calibration_version` /
`final_device_summary` / `safety_stopped`。

`final_state` 映射（`SkillRuntime` 统一装配）：
`Succeeded` / `Failed` / `Cancelled` / `TimedOut` / `SafetyStopped`。

---

## 3. YAML manifest 规范（`skills/*.yaml`）

字段见上表；示例 `hand.open.yaml`：

```yaml
id: hand.open
name: "张开手掌"
version: "1.0.0"
description: "将 O6 六通道位置设为完全张开 {255×6}"
required_resources: ["hand"]
parameters:
  - name: "preset"
    type: "preset"
    required: false
    default: "open"
preconditions:
  - "灵巧手已连接"
execution_stages:
  - "apply_preset"
success_conditions:
  - "张开成功"
failure_conditions:
  - "未连接"
timeout_ms: 5000
cancellation_policy: "controlled_stop"
recovery_policy: "stop_and_report"
safety_profile:
  real_motion: true
preset: "open"
```

加载：`load_skill_manifest(path, descriptor, error)`（`src/skills/SkillManifest.cpp`）——
`id` / `version` / `required_resources` 必填，其余缺省取默认；技能专属数据一并读取。

---

## 4. 运行时流程（`SkillRuntime::run`）

```
resource 互斥（非 dry-run，展开 + 去重）   →  acquire_resources 失败 → Failed
→ validate()（不运动；参数 + 预置条件）    →  validate 失败 → Failed(stage=validate)
→ [dry-run] 短路成功（stage=dry_run）
→ 超时并入 cancel 回调（desc.timeout → 回调返回 true）
→ skill->execute()
→ 结果装配：final_state / 时长 / 安全标志 / 资源释放
```

- **资源互斥**：同一设备同一时刻只有一个 Skill 拥有执行权（`ResourceManager`）。
- **真实运动双重门控**：`--enable-motion`（CLI/ApplicationService）→ `SafetySupervisor::real_motion_enabled()` → `SkillBase::execute` 内 `require_real_motion()`。缺任一 → `safety_stopped`，不运动。
- **取消 / 超时**：统一并入 `exec.cancel` 回调；Skill 在阶段边界轮询 `cancelled(params)` 受控停止。

---

## 5. SkillBase（公共基类，`src/skills/SkillBase.cpp`）

子类只需实现：
- `check_preconditions(const SkillParams&)` —— 预置条件 / 参数校验（不运动）；
- `run(const SkillParams&)` —— 真实执行主体（已过安全门与 dry-run 短路）。

基类 `execute()` 统一：预置条件 → [dry-run 短路] → real_motion 安全门 → `run()`。
辅助：`require_arm/hand`、`require_connected_arm/hand`、`require_real_motion`、
`cancelled()`、`make_ok()`、`make_fail(stage, err)`、`device_summary()`。

---

## 6. 7 个基础 Skill

| id | required_resources | 参数 | 状态机阶段 |
|----|--------------------|------|------------|
| `arm.move_to_safe_pose` | `arm` | `joints`(可选，覆盖 manifest `safe_pose`)、`speed` | validate → move_joint |
| `arm.drag_teach_record` | `combined` | `out_dir`(必填)、`duration_s`(0=直到取消)、`rate_hz`(默认50)、`import`(默认true) | validate → start_drag_teach → start_recording → wait_drag → stop_recording → stop_drag_teach → import_recording |
| `hand.open` | `hand` | `preset`(可选，manifest 默认 open) | validate → apply_preset(Open) |
| `hand.close` | `hand` | `preset`(可选，manifest 默认 close) | validate → apply_preset(Close) |
| `hand.apply_preset` | `hand` | `preset`(必填，open/close/pregrasp/custom) | validate → apply_preset |
| `combined.synchronized_replay` | `arm`,`hand` | `trajectory_id`(必填)、`speed`、`arm_speed_ratio`、`align_start`、`timeout_ms` | validate → 轨迹加载校验 → replay |
| `combined.safe_release` | `arm`,`hand` | `joints`(可选)、manifest `safe_pose` | validate → hand.open → arm.move_to_safe_pose（开始即完成，阶段边界不查取消） |

**`combined.synchronized_replay`**：`check_preconditions` 中加载 + 完整校验轨迹（dry-run 也校验），
`run()` 委托 `TrajectoryReplayer` 按统一时间轴逐点下发 arm.move_joint + hand.set_joint_positions；
支持调速 / 取消 / 超时 / 起点对齐；`ReplayReport` → `SkillResult` 映射（cancelled/timed_out/safety_stopped）。

**`arm.drag_teach_record`**：进入 RM 拖动示教（控制器内记录轨迹），同时经 `RecordingHooks`
同步记录双设备；录制结束后可选 `import_recording` 生成轨迹资产，结果摘要附 `traj=<id>` 供直接复现。

**`combined.safe_release`**：先 `hand.open` 释放抓握，再 arm 回安全位；一旦开始即完成两个阶段
（`finish_current_command` 语义）。

---

## 7. 工厂 / 注册表（`SkillFactory` / `SkillRegistry`）

- `make_skill(desc, ctx, error)`：manifest id → 具体 Skill 实例（未知 id 返回 nullptr + error）。
- `SkillRegistry`：线程安全 `id → ISkill`；`register_skill`（重复覆盖）/ `find` / `ids` / `descriptors`。

---

## 8. SkillContext（依赖注入）

```cpp
struct SkillContext {
    IRobotArm* arm;  IDexterousHand* hand;  ISafetySupervisor* safety;
    IClock* clock;  IStateStore* store;  ITrajectoryRepository* trajectory_repo;
    RecordingHooks recording;  // start / stop / event / active / last_session_dir
};
```

上层（`ApplicationService`）注入领域接口 + 录制钩子；Skill 不直接依赖具体 sink / 文件格式 / 厂商类型。

---

## 9. CLI 用法（`robotctl`）

```bash
robotctl [--real] [--enable-motion] [--skills-dir <dir>] skill list
robotctl [--real] [--enable-motion] [--skills-dir <dir>] skill <id> [params_json] [--dry-run]
```

- `skill list`：列出已注册 Skill（id/version/real-motion 标记/描述）。
- `skill <id> '{"key":"val"}' [--dry-run]`：执行；`--dry-run` 只校验不运动。
- 真实运动必须 `--enable-motion`（CLI + SafetySupervisor 双门控）。
- 参数为 JSON/YAML 对象字符串；缺字段用默认值。

---

## 10. 验收基线

- 构建 / 测试：`mock`(21) / `debug`(23) / `asan`(23) 全部通过（阶段6 新增 4 个测试目标）：
  - `unit_skill_manifest`：YAML 加载（全字段 / 缺 id / 缺 version / 空资源 / 坏 YAML / 缺省值 / 文件不存在）。
  - `unit_skill_runtime`：dry-run 短路、validate 失败、成功执行 + 资源释放、资源冲突、combined 展开去重、取消映射、超时映射、失败映射。
  - `unit_skill_basic`：hand.open/close/apply_preset（含参数覆盖与非法预设）、arm.move_to_safe_pose（manifest/参数两种来源）、安全门拒绝（手不动）、safe_release（手张开+臂安全位）、drag_teach 缺 out_dir。
  - `integration_skill_combined`：load_skills 注册 7 个 → 录制 → 导入轨迹 → synchronized_replay → safe_release → 未注册 id 查找失败；dry-run 不运动。
- Mock 手工链路：见 `session6.md` §3。

---

## 11. 相关文档

- `Prompt.md`：Skill 定义 §13、动作分级 §561-597、阶段6 §1214。
- `session6.md`：阶段6 执行版（交付清单 / 编译坑 / 验收 / 阶段7 入口）。
- `docs/implementation_plan.md`：阶段6 ✅ 完成记录。
- `docs/architecture.md`：分层架构。
- `docs/trajectory_format.md`：轨迹格式（`combined.synchronized_replay` 依赖）。
- `session5.md`：阶段5 执行版（轨迹管理）。
