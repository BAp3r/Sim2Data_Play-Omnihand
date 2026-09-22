# Sim2Data 验证记录

## 2026-09-22 下一阶段实施（基线 0e523e0）

用户本轮明确恢复 A/B/D 及有条件的运行时验证。实施分支从已发布 `0e523e06bb5b0909640a5875e0c37e42e7c4eec6` 派生，旧 A/D/E worktree 保留；不发布、不合并 main。实际启动三个 Luna Max 子代理，工作区和写入范围独立；主会话复核视觉观察、维护公共接口并集成。

### B 装配输入与坐标链

消费主会话实际审阅的 `OBS-M1-20260922-ROOT-01`；原片/帧哈希、时间位置和批准范围存 `.local/evidence/m1/approved_observation.json`。画面仅支持布局/安装拓扑，双 D405 型号来自用户说明，米制外参未解析。

交付 `configs/assembly_inputs.template.json`、`docs/ASSEMBLY_INPUTS.md` 与手根/腕相机原子链接口。B 分支提交 `d251fb5c03bf6f587673c74c8d25ced2a2012bfa`，主分支审阅后 cherry-pick 为 `bdd9a64`；集成时进一步明确每侧安装载荷和模板不直接作为 scene_spec 的边界。B 单独测试实际为 **65 项总数、63 通过、2 跳过**，不是 65 通过外加 2 跳过。

主会话复查：

```powershell
uv run --frozen --offline --no-python-downloads python -m unittest tests.test_frames tests.test_runtime_identity -v
uv run --frozen --offline --no-python-downloads python -m sim2data.preflight --out .local/evidence/m1/preflight_after_assembly.json
```

18 项目标测试通过。预检退出 2，64 个未解析声明、0 个固定契约冲突、采集关闭。证据：`assembly_identity_tests.txt`、`preflight_after_assembly.json`。没有场景投影、初始碰撞、机器人运动、可达性或 M2 验收。

### 运行时身份与实际启动失败

三套候选用 `scripts/runtime_identity.py` 实际重新检查；解释器/包来源、editable 源码、继承配置和选定哈希见私有 `.local/evidence/m1/runtime{51,50,61}.json`。5.1/5.0 的 Isaac Lab 源码 release 文件是 2.3.0、包元数据 0.47.3，没有独立 Git 身份；三个核心文件与官方 v2.3.0 相同，但扩展元数据不同。不能据此声称整份源码原版或生产锁已完成。具体绑定方案见 `RUNTIME_BINDING.md`。

远端准确命令包含机器路径，存 `.local/evidence/m1/COMMANDS.md`。实际命令形态为：

```bash
uv run --no-project --offline --no-python-downloads --python <existing-python> python runtime_identity_review.py --out <new-private-report.json>
OMNI_KIT_ACCEPT_EULA=YES timeout 240s uv run --no-project --offline --no-python-downloads --python <existing-isaac51-python> python isaac_smoke.py --asset synthetic_box.usda --asset-role synthetic_fixture --out m1_synthetic_smoke_01
```

第一次 Kit smoke 使用自建 8 cm 立方体隔离启动，不是 card_box 或机器人。启动前 GPU 0% / 8248 MiB，占用允许尝试；日志出现重复 Vulkan NVIDIA ICD 枚举、默认共享 DerivedDataCache 锁失败和 RTX 段错误，**进程退出 139，physics/RGB 未通过**。崩溃发生在 SimulationApp 初始化，未产出有效 RGB/轨迹。日志及当时脚本分别为 `m1_synthetic_smoke_01.log`、`isaac_smoke_first_attempt.py`。

后续按安装包源码改为显式 `--portable-root` 和单 GPU 配置，只通过 py_compile，未复试。其他任务随后占用 81777 MiB / 100%，本会话未停止其他进程；运行时继续阻断。单进程选择已核对的唯一 Vulkan ICD 是下次诊断方案，不是已验证修复。未修改驱动/系统包/共享环境，也未手工修改或清理公共 cache。

下面章节为旧阶段历史，不能覆盖本节的最新实施授权与分项结果。


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
