# M0 验证记录

## 2026-09-22 本地轻量复核（文档交付后）

输入基线为本地提交 `f9509ba`，Windows / uv / CPython 3.12.11。范围为已合入的 M0 框架；A/D/E 在途 worktree 未合入，也未恢复 GPU、网络资产检索或官方 SDK 测试。另由一个实际启动的 Luna Max 只读子代理审查文档、配置与 B 任务卡的一致性；主会话裁决并修改公共接口。

先复现再修复：新增失败用例发现布尔时间戳被当作 0、通道名列表可被外部修改、单字符串可冒充通道向量；预检未明确报告单位/任务顺序冲突；草案缺少手资产和原子法兰/相机安装链。文档审阅同时补充了 RGB/depth 分界、synthetic 主相机必填位姿、中转门禁字段和批准观察包要求。

实际运行命令：

```powershell
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
uv run --frozen --offline --no-python-downloads python -m compileall -q sim2data scripts tests
uv run --frozen --offline --no-python-downloads python -m sim2data.preflight --out reports/local_review_preflight.json
```

结果：修复后 **59 项测试，57 通过，2 跳过**；跳过仍为 Windows 软链接权限相关用例。compileall 返回 0。预检返回预期的 2，报告 64 个未解决字段、0 个固定契约冲突、`production_collection_allowed=false`。64 是字段数，不表示要用户手工提供 64 次测量；大量字段需从模型、接口或已有标定解析。CLI 用例还确认草案只读及旧报告不被覆盖。

补充只读检查：用同一 uv Python 的 `json.loads` / `tomllib.loads` 解析 3 个 JSON、7 个 TOML，68 个必需路径均存在于草案；针对 7 个私有路径执行 `git check-ignore --no-index -q -- <path>`，针对 USD/USDA/USDC/USDZ/BLEND 执行 `git check-attr filter -- <path>`，均符合配置。`git ls-files -z` 中未发现上述私有目录的已跟踪文件。此结果仅覆盖指定目录排除和 LFS 属性，不是完整凭据/许可审计，也不是 LFS 上传或拉取验证。

私有证据：`reports/local_contracts_before_fix.txt`、`local_contracts_after_fix.txt`、`local_repository_checks.json`、`local_review_preflight.json`。未执行：Isaac/PhysX、真实/仿真相机投影、机器人单通道运动、接触接力、中转门禁的物理判定、LeRobot 官方回读、批量写入和吞吐。剩余阻塞按 `EXECUTION_PLAN.md` 的资产、装配和运行时关卡推进。生产采集持续关闭。

## 2026-09-22 状态更新

用户最新要求：本轮优先完成设计、计划与 AGENTS.md，新增测试后置。以下历史结果保留，不能据此宣称物理抓取或生产数据集兼容。

本机 Windows 已在用户指示暂停前执行：

```powershell
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
uv run --frozen --offline --no-python-downloads python -m compileall -q sim2data scripts tests
uv run --frozen --offline --no-python-downloads python -m sim2data.preflight --out reports/preflight_current_20260922.json
```

最近一次集成单元检查包含 47 项：45 项通过，2 项因 Windows 软链接权限跳过。compileall 在当时的 root 框架版本通过；预检返回 2 并阻止 M0 采集，符合当前阶段预期。原始输出在 ignored `reports/`，不包含新近未合入的 A/D/E 在途实现。暂停后只修改文档，未重新运行测试。

已通过只读 SSH/NAS 访问并记录现有运行时版本；未启动 Kit/GPU 仿真、真机控制或正式数采。LeRobot 官方写入回读、场景打开、接触接力、远程 LFS 与吞吐均不在本轮已验收范围。

## 2026-09-21 历史沙盒记录

验证位置是本会话的 Linux 沙盒，不是用户的 Ubuntu PRO6000 服务器。

已运行：

```bash
uv lock --offline
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
uv run --frozen --offline --no-python-downloads python -m compileall -q sim2data scripts tests
```

30 项单元测试通过。测试原始输出见 `TEST_RESULTS.txt`。JSON 场景配置和 TOML 子代理模板已通过语法解析。测试解释器是 CPython 3.13.5；这里不是仿真运行环境版本建议。

`collect_inventory.py` 已在沙盒执行，验证了单文件入口；未在 Windows、用户 NAS 或用户 Ubuntu 服务器运行。扫描异常、截断和输出覆盖策略有单元测试，Windows UNC 的实际访问仍须在用户机器核验。

测试覆盖：时间步契约、可重复子种子、资产路径逃逸/缺失/LFS pointer、具名 state/action 维度与数值、相机时间标签一致性、盘点结果截断和报告覆盖。

不包含：Isaac/PhysX 运行、真实相机渲染同步、接触抓取、LeRobot SDK 写入回读、LFS 推拉、SSH 登录、子代理实际启动及 GPU 吞吐。元数据时间测试不能证明渲染器的真实图像没有延迟。

交付中的视频关键帧和 `.local/DEPLOYMENT.md` 供本项目私有使用；不要通过公有 GitHub 分享。
