# 线程模型 (Threading Model)

> 对应 Prompt.md 第七节。描述一期模块化单体架构下的执行单元划分与并发约束。

## 1. 执行单元

| 单元 | 线程数 | 职责 | 关键约束 |
|------|--------|------|----------|
| Main / Application | 1（主线程） | CLI 解析、系统启停、命令分发 | 不执行周期采集 |
| RM75 状态采集 | 1 | 拉取/接收机械臂状态 → 转换 → 写入 StateStore | 不写文件、不执行 Skill |
| O6 状态采集 | 1 | 拉取灵巧手状态 → 转换 → 写入 StateStore | 不写文件 |
| CommandScheduler | 1 | 同一设备运动命令串行化、命令生命周期、取消/超时 | 一次一个命令 |
| SafetySupervisor | 1（或定时触发） | 检查新鲜度/限位/速度/力/温度/通信/权限 | 必要时请求停止 |
| Recorder 写盘 | 1 | 从有界队列消费记录，写文件 | 不阻塞采集线程 |
| SkillRuntime | 1（或任务执行器） | 执行 Skill 状态机 | 不直接操作串口/SDK |

## 2. 数据流

```
RM75 SDK ──→ RM75采集线程 ──→ StateStore (不可变快照)
O6 SDK   ──→ O6采集线程   ──→ StateStore
                                     │
                                     ├──→ SafetySupervisor（评估）
                                     ├──→ Recorder 队列 → 写盘线程 → CSV/文件
                                     ├──→ CLI status / doctor（读取）
                                     └──→ SkillRuntime（阶段6）
```

## 3. 并发设计要点

1. **状态数据采用不可变快照**：`RobotArmState` / `DexterousHandState` 为值类型，
   写入方构造新副本，读取方持有副本不受后续写入影响。
2. **共享状态用明确同步**：`StateStore` 内部 `std::mutex` 保护；
   `CommandScheduler` 用互斥量 + 条件变量；`Logger` 用互斥量。
3. **同一设备命令串行化**：所有运动命令经 `CommandScheduler` 单线程执行，
   避免多个模块同时向同一设备发命令。
4. **文件写盘不持有设备锁**：Recorder 只从有界队列消费，不反向阻塞采集线程。
5. **日志不持有设备控制锁**：`Logger::log` 只保护自己的输出缓冲。
6. **回调中不等待另一设备**：采集线程只做「获取→转换→入队」，不做设备间等待。
7. **所有线程受控停止**：`CommandScheduler::stop()` 清空队列并等待当前命令结束；
   程序退出按顺序：停止 Skill → 停止记录 → 停止设备运动 → 断开设备。

## 4. 一期实现状态

- 阶段1（当前）：`CommandScheduler` 已实现单线程串行化；`StateStore` 线程安全。
- 阶段4：新增 RM75/O6 采集线程 + Recorder 后台写盘线程。
- 阶段6：新增 SkillRuntime 线程。

## 5. 厂商回调线程约束

- RM75 UDP 推模式回调（`rm_realtime_arm_state_callback`）在 SDK 内部线程调用，
  回调体内**不得执行耗时业务逻辑**，只做快照转换。
- O6 SDK 的 TX 回调可能从 SDK 内部工作线程调用（`sendWorker`），回调体必须线程安全。
- 不允许在厂商 SDK 回调线程中写文件或执行 Skill。
