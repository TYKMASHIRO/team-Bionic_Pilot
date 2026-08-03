# 协作流程

## 分支规则

- `main` 是稳定主分支，只接受 Pull Request 合并。
- 每个功能、修复或明确任务单独创建一个分支和一个 Pull Request。
- 正式成员功能分支使用 `feature/<功能名>`。
- Bug 修复分支使用 `fix/<问题名>`。
- 实习生功能分支使用 `intern/<功能名>`。
- 已创建的 `dev/person-1` 和 `dev/intern-review` 可作为个人同步分支；实际开发仍建议按功能再拆分分支。

## 合并规则

- 所有合并到 `main` 的代码都必须通过 Pull Request。
- 所有 Pull Request 都必须请求 `@TYKMASHIRO` 审查。
- `@TYKMASHIRO` 审查通过后再合并。
- 一个 Pull Request 不混合多个无关功能。

## 推荐流程

1. 从最新 `main` 创建功能分支。
2. 在功能分支完成单一功能或修复。
3. 推送分支并创建 Pull Request 到 `main`。
4. 请求 `@TYKMASHIRO` 审查。
5. 审查通过后合并到 `main`。
