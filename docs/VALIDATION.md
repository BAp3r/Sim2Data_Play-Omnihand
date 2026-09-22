# Sim2Data 验证记录

## 2026-09-22 本轮集成结果

输入为已发布 M0 基线；本地实施分支 `work/m1-implementation-20260922`，未推送。包迁移提交 `b42eb75`；A 官方 API 取证集成为 `38a873b`；Astra CAD 审查原提交 `44bb93c`、集成为 `e1e4427`。后续文档提交见 Git 历史，不把未提交状态填作 SHA。

### 源码布局与环境锁

实际固定官方 submodule：Isaac Lab v2.3.0 / 包版本 0.47.2，commit `3c6e67bb5c7ada942a6d1884ab69338f57596f77`；LeRobot 0.6.2，commit `b64fe1ed9f11eeac53ee821356d2797601701054`。三包 editable 导入保持 `sim2data.*` 接口；文档见 `PACKAGING.md`。

实际运行：

```powershell
uv lock --check --offline
uv lock --project environments/data --check --offline --no-python-downloads
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
uv run --frozen --offline --no-python-downloads python -m compileall -q packages scripts tests
```

根锁 4 包、data 锁 64 包检查通过；测试 **77 项总数、73 通过、4 跳过**（2 个 Windows symlink、2 个本机未安装 SDK 的集成测试）。compileall 通过。原始输出 `.local/evidence/m1/final_cpu_tests.txt`。没有通过本机跳过项宣称 SDK 兼容。

sim 锁在隔离 Linux 目录由真实 `uv lock --project environments/sim --python <existing-python> --no-python-downloads` 解析 **200 包**，随后 `uv lock --project environments/sim --check --offline --no-python-downloads` 返回 0。最初 Windows 在线解析因 NVIDIA 大 wheel 范围读取/内存映射失败；Windows 离线复核也因缺元数据缓存失败。仅在 Linux 的实际解析和检查记为成功。旧 flatdict 构建依赖 `pkg_resources`，隔离构建约束 `setuptools<81`；Isaac Lab 构建补 `toml`。没有完整 sync 两套环境，锁解析不是物理验收。精确私有命令和日志见 `.local/evidence/m1/COMMANDS.md`、`sim_lock_linux_retry.log`、`sim_lock_check.log`。

最终预检 `uv run --frozen --offline --no-python-downloads python -m sim2data.preflight --out .local/evidence/m1/final_preflight.json` 仍阻断：64 个未解决字段、0 个固定契约冲突、`production_collection_allowed=false`。6 个受跟踪 JSON、11 个 TOML 解析通过；两个 submodule 工作树干净；已跟踪文件指定私有地址/路径字面量扫描无命中，私有目录无跟踪项。检查记录 `final_repository_checks.json`；这不是完整第三方许可或凭据审计。

### A 资产与 SDK 静态证据

`configs/asset_manifest.json` 和 `ASSET_AUDIT.md` 是下一阶段绑定入口。NAS 官方 card_box 的实际 USD 为 0.70×0.50×0.50 m、米制/Z-up、boundingCube collision、无 authored rigid body/质量；七个显式依赖存在，未验证渲染解析。桌面应用需先审阅缩放和动态 wrapper/物理参数。

AIRBOT 两个指定候选复用旧快照并复核哈希，未将第三方仓库称官方；mesh、实物修订及 motor 映射待绑定。OmniHand 官方 O10 页面和 commit `026740d9fdd8ba32b0605fa702a992b322076f1b` 的 API 文档实际取回，确认 10 active + 6 passive 及具名主动角度表；16 项角度接口不是已冻结 observation 维度。官方模型 archive 尚未做 hash 绑定，SDK release version 未确认。详细哈希/取证命令在上述审计文档和私有 evidence index。

### B / Astra 单件 CAD

用户指定 STEP 经实际启动的 GPT-6 Astra High 解析审阅，观察 `OBS-M1-20260922-ASTRA-FLANGE-01`。1 个有效 solid、134 面；约 37.568×26.100×37.568 mm。CPU BRep/mesh 尺寸与体积核对通过；完整输入版本、准确命令、哈希和派生几何保存在 `.local/evidence/m1/flange/`，公共报告 `FLANGE_ASSEMBLY_REVIEW.md`。没有 Blender 导入或 Isaac cooking/GPU 验证。

模板只新增批准观察引用，没有把 CAD 长度或原点偏移填入生产变换。两端归属、搭接、datum、材料/质量、D405 支架和光学外参仍缺。B 原子链、预检及 scene requirements 是下一接入接口；M2 不通过，采集继续关闭。

### D 官方 LeRobot 0.6.2 synthetic 回读

D 原提交 `2c9813b` 集成为 `703814c`。固定上游 b64fe1ed 的真实源码通过私有 packaging shim 构建 wheel，以 `--no-deps` 放入独立 overlay，未修改共享环境。实际解释器 Python 3.12.13，Torch 2.11.0、NumPy 2.3.1、PyAV 15.1.0；完整身份、源码/包哈希、依赖和命令保存在 `.local/evidence/m1/export_official/`。这是官方源码 + 本次已有依赖组合的验证，不是原样上游 wheel 安装测试，也不是新 data 锁全量 sync。

通过官方 `create/add_frame/save_episode/finalize` 写入，官方 loader 实际重开：2 episodes / 6 frames、三个 8×8 H.264 RGB、每路 2 个视频文件；分片边界解码、统计、episode 局部窗口/padding、sidecar、Torch batch 检查成功。窗口 state/action batch 为 `[2,2,2]`；overhead 窗口 batch 为 `[2,2,3,8,8]`。D 快照 63 项测试全部通过，其中 2 项 SDK 集成测试亦独立运行通过；它与根迁移后 77 项本地测试范围不同。

准确私有命令存证据目录；公开可复用命令形态（先安装本项目包/显式设置审核过的 overlay）：

```bash
uv run --no-project --offline --no-python-downloads --python <verified-sdk-python> python -m unittest tests.integration.test_lerobot_v3 -v
uv run --no-project --offline --no-python-downloads --python <verified-sdk-python> python scripts/lerobot_smoke.py --root <new-private-root> --report <new-private-report.json>
```

`source_identity.json`、`tests_official.stdout`、`full_tests.stdout`、`uv.stdout`、`uv_report.json`、`dataset_inspection.json` 提供原始结果。旧 0.5.2 checkout 无法从官方取回 commit，仅作兼容分支测试；0.4.3 的两个兼容测试同样不替代当前固定源证据。未上传 Hub、未共写数据根、未手拼 v3 元数据。**格式小样通过，真实机器人 M4 未通过。**

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
