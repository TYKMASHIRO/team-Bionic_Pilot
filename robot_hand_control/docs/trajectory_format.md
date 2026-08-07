# 轨迹格式 (Trajectory Format) — 阶段5

> 本文档定义轨迹（Trajectory）的数据格式、磁盘布局、加载/校验/复现语义。
> 轨迹是"录制 → 导入 → 校验 → 复现"链路中的核心对象。

## 1. 设计原则

- **复用录制契约**：轨迹直接复用 `states.csv` 的 110 列固定列序（唯一来源 `CsvStateColumns`），
  保证录制写出的列与轨迹读回的列永远一致，无两套列定义。
- **统一时间轴**：`t_offset_ns` 为相对首帧的单调偏移（ns），所有点按该时间轴推进，与录制墙钟解耦。
- **双设备一体**：一个轨迹点同时承载 `RobotArmState`（arm）与 `DexterousHandState`（hand），
  复现时按统一时间轴同步下发。
- **不做假数据**：字段缺失用 `valid=false` 缺席编码表达，不复造厂商 API 语义。

## 2. 磁盘布局

轨迹资产目录（由 `TrajectoryRepository` 落盘）：

```
<data_dir>/<trajectory_id>/
├── states.csv      110 列固定列序（CsvStateColumns），表头 + 数据行
├── events.csv      事件行（可选；与录制 events.csv 格式一致）
├── metadata.txt    录制格式版本 / config_hash / calibration_ref
└── manifest.txt    轨迹专属字段（trajectory_id / name / source_recording / created 等）
```

其中 `<data_dir>` 默认 `data/trajectories`，`<trajectory_id>` 由 `import_recording` 生成为
`traj_<session_id>`（同一录制目录重复导入幂等覆盖）。

### 2.1 states.csv

- 表头行 + 每帧一行，`CsvStateColumns::count()` == 110 列。
- 列序与阶段4 `CsvRecordSink` 写出的 states.csv **完全一致**（全局 8 + arm 56 + hand 46）。
- 缺席编码：设备 `valid=false` 时数值列写 `0`、字符串/矢量列写空串、`*_valid=0`。
- 消费方以 `*_valid` 判定设备在场，不依赖其他列猜测。

### 2.2 events.csv

行格式（与录制一致）：

```
iso8601, wall_ns, steady_ns, event
```

- `event` 经 CSV 转义（逗号/引号/换行处理）。
- 加载时事件按 `steady_ns` 关联到轨迹上 `t_offset_ns >= (event_steady_ns - 首帧_steady_ns)`
  的第一个点；一个点上多个事件以 `;` 连接。

### 2.3 manifest.txt

`key=value` 行式文本（无 YAML 依赖），字段：

| key | 含义 |
|-----|------|
| `trajectory_id` | 轨迹 id（`traj_<session>`） |
| `name` | 轨迹名（可选） |
| `source_recording` | 来源录制 session id |
| `calibration_id` | 标定引用 |
| `config_hash` | 录制配置 hash |
| `version` | 轨迹/录制格式版本 |
| `sample_count` | 状态点数 |
| `duration_s` | 时长（秒，末帧-首帧偏移） |
| `created_wall_ns` / `created_steady_ns` | 创建时间 |

## 3. 加载（TrajectoryCsvLoader）

`TrajectoryCsvLoader::load_recording(recording_dir, Trajectory&)`：

1. 读 `states.csv`；表头列数 != 110 报 Validation 错。
2. 逐行 `CsvStateColumns::parse_row`；任一数值列解析失败报错并给出行号。
3. `t_offset_ns = steady_ns - 首帧_steady_ns`（首帧为 0，单调）。
4. 关联 `events.csv`（见 2.2）。
5. `metadata.txt` 存在时校验 `format_version == kRecordingFormatVersion(1)`。
6. 填充 `TrajectoryMeta`：`source_recording`=目录名、`version`、`sample_count`、
   `created`=首帧时间戳、`duration_s`、`config_hash`、`calibration_id`。

## 4. 校验（TrajectoryValidator）

`TrajectoryValidator::validate(traj, report, limits)`：

| 项 | 规则 | 失败类别 |
|----|------|----------|
| 非空 | points 非空 | error |
| 版本 | `version` 存在时须 == 1 | error |
| 关节数 | arm 7 / hand 6（强类型下恒匹配，保留防御检查） | error |
| 时间戳单调 | `t_offset_ns` 非递减 | error |
| NaN/Inf | arm/hand 全部数值列有限 | error |
| 范围 | arm 关节位置 ∈ [-3.5, +3.5] rad；hand raw ∈ [0, 255] | error |
| 双设备在场 | 全轨迹无有效 arm/hand 状态 | warning |

`TrajectoryValidationReport`：`valid/errors/warnings/sample_count/duration_s/monotonic/finite/in_range/dof_ok`。

## 5. 复现（TrajectoryReplayer）

`TrajectoryReplayer::replay(traj, options, cancel)`，严格顺序：

1. **校验**：`TrajectoryValidator`（不运动）。
2. **设备在线**：arm/hand 已注册且已连接。
3. **起点偏差**：arm 当前关节与轨迹首点 arm 关节的最大绝对偏差（rad）。
   - `--dry-run` 在此返回：报告偏差，不运动。
4. **安全评估**：真实运动前 `ISafetySupervisor::evaluate(combined)`；
   未启用 `--enable-motion` 或状态过期/超限即拒绝（`safety_stopped`）。
5. **起点对齐**：偏差 > `start_deviation_tol_rad(0.1)` 时：
   - `align_start=true` → 先 `move_joint(首点, block=true)` 回起点；
   - `align_start=false` → 直接拒绝（`start_alignment`）。
6. **播放**：按统一时间轴逐点下发：
   - arm：`move_joint(position, speed_ratio, block=(末点))`；
   - hand：`set_joint_positions(position)`；
   - 每点前检查 cancel 回调与总时长 timeout；
   - 推进到 `t0 + t_offset_ns / speed`；下发耗时超间隔时不补眠（跟随落后）。

### 5.1 调速语义

`options.speed` 为**时间轴缩放倍率**：目标时刻 = `t0 + t_offset_ns / speed`。

- `speed = 2` → 两倍速（时间轴压缩一半）；
- `speed = 0.5` → 半速；
- `speed <= 0` → 兜底按 `1.0`（原速）。

不做轨迹插值/重采样：播放仍逐点，仅缩放点间等待时间。

### 5.2 执行报告（ReplayReport）

| 字段 | 含义 |
|------|------|
| `success` | 是否成功完成（未被取消/超时/失败） |
| `error` / `failed_stage` | 失败原因与阶段（validate/devices_online/dry_run/safety/start_alignment/play_arm/play_hand/stopped） |
| `points_played` / `total_points` | 已播放 / 总数 |
| `duration_ms` | 总耗时 |
| `cancelled` / `timed_out` | 是否取消 / 超时 |
| `dry_run` / `safety_stopped` | 是否为校验模式 / 安全停机 |
| `start_deviation_rad` | 播放时实际起点偏差 |
| `final_summary` | 设备最终状态摘要（arm 关节 / hand 位置） |
| `*_version` | 轨迹/配置/标定版本 |

## 6. CLI 用法

```
robotctl trajectory import <recording_dir>        # 导入为 traj_<session>
robotctl trajectory list
robotctl trajectory inspect <id>
robotctl trajectory validate <id>
robotctl trajectory replay <id> [--speed <x>] [--dry-run] [--enable-motion]
robotctl --data-dir <dir> ...                      # 轨迹数据目录（默认 data/trajectories）
```

- 复现**真实运动必须** `--enable-motion`（`SafetySupervisor` 门控）。
- `--dry-run` 只校验不运动（仍需设备在线，验证起点偏差/设备/安全前置）。
- 复现中 Ctrl-C → cancel 回调 → 提前停止并报告 `cancelled`。

工具：`trajectory_inspector <dir> [--validate]` 检查录制/资产目录能否加载为轨迹。

## 7. 数据流总结

```
record (CsvRecordSink → states.csv)
   ↓
import (TrajectoryCsvLoader → Trajectory; TrajectoryRepository 落盘资产)
   ↓
validate (TrajectoryValidator)
   ↓
replay (TrajectoryReplayer → arm.move_joint + hand.set_joint_positions)
```
