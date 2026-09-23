# Sim2Data 验证记录

## 2026-09-23 完整装配静态 RTX smoke（M7 实施证据）

新增入口 `scripts/isaac_scene_smoke.py`，在独立 Windows Isaac Sim 5.1.0.0 / Isaac Lab 0.47.2 环境中读取审阅过的双臂 USD、NAS cardbox 与 Thor 源 USD。脚本启动前校验 preview report、profile、纹理和 manifest SHA256；通过 session layer 重新绑定平台可读的 NAS 桌面路径，不修改源资产。结果写入 ignored `.local/evidence/m7/full_scene04/`，三路原相机均实际产生 `640×480×3` RTX RGB：`overhead.png`、`wrist_left.png`、`wrist_right.png`。主视角显示双臂、OmniHand、Thor 桌、纸盒和右侧框子；腕相机当前限位姿态只看到局部指尖/背景，覆盖不接受。

静态装配的 schema inventory 为 0 个机器人 rigid body、0 个 articulation，Thor 引用含 1 个碰撞 prim；本 smoke 不推进 PhysX、不创建机器人 drive、不执行接力。纸盒仅作为静态视觉引用，盒子落体/质量 wrapper 的独立证据仍是 `.local/evidence/m6/cardbox04_nas_drive/`。因此本轮通过范围是“完整 USD 可打开 + 三路 RTX 帧可产出”，不是物理、机器人驱动、同步、标定或抓取验收。

Kit 关闭采用 `fast_shutdown=false`、`skip_cleanup=false`，但在扩展清理阶段发生访问冲突，外部进程退出码 `-1073741819`（0xC0000005）；`full_scene04/shutdown_stack.txt` 和 `kit.log` 保留了关闭栈，定位到 stage 已关闭后 `omni.usd` 的扩展卸载。主会话根据外部退出证据给结果 JSON 补注 `shutdown=kit_close_access_violation`，不能写成正常退出。CPU 回归为 98 项，90 通过、8 跳过；compileall、`git diff --check` 和五项新增输入身份测试通过。

## 2026-09-23 Pinocchio 规划运动与 LeRobot 单 episode

用户选择先交付规划运动样本，不要求本轮伪造完整接力成功。远端现有环境实际导入 Pinocchio 4.0.0，用用户 play URDF 做六轴位置 IK：左腕上移 25 mm 后返回，右臂保持；121 帧、30 Hz、4 秒，IK 残差 `7.997e-8 m`，峰值有限差分速度 `0.08311 rad/s`。初始超速度候选被拒绝。没有安装或调用 cuRobo；没有把零 effort/velocity 改成控制限值。

本机独立 Isaac 5.1 / RTX 3080 以每帧 FK 更新真实 USD 并渲染三路 `320×240` RGB。最终输出 `capture02/capture.json` 共 121 帧，三路均 121 个不同图像，并保留每帧相机位姿与 synthetic K；Pinocchio→USD 世界变换最大误差 `7.77e-16`。没有推进 PhysX，没有机器人 articulation/drive、碰撞或抓取成功判定。证据与对照图在 ignored `.local/evidence/m8/`。

复用已审计的现有 Linux 解释器与私有官方 SDK overlay（并非新建独立数据 venv），使用 LeRobot 0.6.2、codebase v3.0 writer：最终 `dataset03` 恰好 1 episode/121 帧，三路视频各 121 帧，state/action 12 维，训练 batch 形状分别 `[2,12]` 与 `[2,3,240,320]`。官方 loader 对全部 121 帧回读后，state/action 最大误差 `2.97e-8`、时间戳最大误差 `1.11e-7 s`，视频逐帧解码；各流最大全帧平均 RGB 压缩误差为 1.14～1.36/255。完整报告为 `.local/evidence/m8/full_readback03.json`，SDK 原始报告为 `.local/evidence/m8/export_report03.json`。sidecar 保持 `task_success=null`、`physics_validated=false` 和 synthetic adapter timeline 语义；首轮不含逐帧相机位姿的 dataset02 保留为诊断产物。

最终轻量测试104项、96通过、8跳过；compileall通过。新增五项输入身份测试与六项运动导出门禁测试均通过。静态 `full_scene05` 释放 Python USD 引用后仍在扩展清理访问冲突，运动 `capture02` 写完也以同一码退出；两者的 `process_exit.json` 与关闭栈单独保留，正常关闭尚未修复。实际启动两名 GPT-6 Luna Max：一名只读 smoke 审查，一名独立 worktree 导出实现；导出原提交 `bf4a3819e8f2927e7f14bebcd45404c3cd817143` 按文件审阅集成，实际 SDK 执行由主会话完成。

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

官方源码补充复核：overlay 内 538 个 Python 文件与固定 b64fe1ed submodule 逐文件 SHA256 一致，无缺失或新增 Python 文件；packaging shim 仅改打包元数据。私有证据：`export_official/source_python_hash_compare.json`、`local_submodule_crosscheck.json`、`lerobot_packaging_shim_setup.py`、`commands.json`。这不扩展到依赖环境或上游完整构建流程的验收。

## M2 本机调试增量

`uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v`：85项，81通过、4跳过，日志 `.local/evidence/m2/cpu_tests03.txt`。两次 Blender CPU 静态导入/三相机渲染完成，证据 `blender_review01` / `blender_review02`，不是物理或数据回合。新环境首轮 `newenv_smoke01/result.json` 明确失败：physics context 未初始化；修正生命周期后复试。三路同步、机器人动力学、接触任务均未验收，生产关闭。

## 本机 headless 物理/RGB 结果（2026-09-22）

新隔离 Windows 环境（Isaac Sim 5.1.0.0、IsaacLab 0.47.2/v2.3.0、Torch 2.7.0+cu128）已实际完成 synthetic 8 cm Cube 的480步测试，dt=1/240 s、CPU PhysX + D3D12 RTX RGB。中心高度从0.289489 m降至0.03999999 m，末速度约0.000176 m/s，RGB为240×320×3，标准差39.2979；主会话已查看实际图像。`newenv_smoke02/result.json` 为 passed=true / completed。该结果仅证明桌面夹具下落支撑和单相机渲染，不包含官方 card_box、机械臂、三相机同步或接力数据。Kit 首次着色器编译较慢，物理结果写完后关闭阶段另行记录，不以退出码替代结果检查。

精确机器命令与私有证据见 `.local/evidence/m2/COMMANDS.md`。装配复核见 `ASSEMBLY_VISUAL_REVIEW.md`；生产开关保持关闭。

关闭阶段补充：结果写完后 Kit 超过4分钟未退出，主会话核对进程命令后仅终止本次 smoke。物理/RGB通过，正常关闭未通过；证据 `.local/evidence/m2/newenv_smoke02_shutdown.json`。Blender review04 改用独立灯光/曝光，已实际查看；它对应 camera v1 位置的 preview02，不能用作公开 v2 偏置的复验。

## 用户单臂与Thor桌面修正验证

输入：用户play URDF `b02c7ac6...`，7个STL闭合；官方Thor `table_instanceable.usd` 与实例依赖只读审计。源码绑定、单位、末端几何与来源限制见 `ASSEMBLY_CORRECTION.md` 和 `THOR_TABLE_AUDIT.md`。`uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v`：88项，84通过、4跳过；日志 `.local/evidence/m3/cpu_tests.txt`。新增测试覆盖预览限位默认值、非法覆盖及mimic超限。

`assemble_commissioning.py` 用用户包生成双侧URDF成功；同一profile用旧ROS2 binding会报source identity mismatch且不创建输出目录。远端CPU `build_commissioning_preview.py --table-usd ...` 实际生成并重新打开200 prim静态场景，三路Camera保留，Thor源尺度未改变，profile哈希 `a5f919aa6b2b383b9f073d0cb4cfdf163b07c3bdbbf4c5454420d3aefc057c16`。Blender实际导入，生成三路静态视图及左右腕部近景；所有图仅用于审阅。

未运行：本次新机器人和Thor组合的PhysX cooking、动力学、接触、相机覆盖与数据导出。六轴effort/velocity为零，连接件机械配合与相机支架缺失，Thor视觉/碰撞支撑高度不同；生产继续关闭。精确命令、输出位置和图例见私有 `.local/evidence/m3/COMMANDS.md`。

## 左右对称与参考配色验证

Astra xhigh实际检查独立左右手URDF及36个源mesh；右实体D405滚转候选使外壳中心/光心镜像残差降至浮点误差范围，此数值不是实测精度。有效静态姿态下全visual AABB分离，自遮挡采样左18/315、右20/315，中心射线畅通；详见 `WRIST_SYMMETRY_REVIEW.md`。

CPU测试89项，85通过、4跳过；日志 `.local/evidence/m4/cpu_tests.txt`。新增回归验证名义外壳与光心对称，以及右optical X/Y反向、Z同向。误将左手URDF绑定右侧的失败注入被SHA校验拒绝，未创建输出目录。USD新旧54个机器人mesh的points/counts/indices逐字节哈希一致，变换和材质按设计更新；参考色不是重建几何或原厂纹理。

远端CPU生成带材质分区的静态USD，本机Blender实际渲染三路静态相机、左右近景、双腕正面和俯视共7张审阅图。首次单子集导入缺少base fallback，已改显式完整材质partition并重新渲染；不改变网格。精确命令和证据在 `.local/evidence/m4/COMMANDS.md`。本轮未执行物理、接触、动态任务覆盖或数据导出，生产保持关闭。

## 实录外观与D405外移复验（2026-09-22）

同一Astra xhigh在基线388382b上修正显示材质和synthetic位置。输入继续使用已绑定用户play、独立左右OmniHand、官方D405及Thor；新实录只支持外观观察，不提供机械标定。左右housing的mount Y分别从±.011到±.031 m，保留旋转与内部光学链。实际FK世界X增量为左-0.01999999999964、右+0.01999999999964 m；源URDF近似角产生约0.119微米Y和0.0054微米Z分量，属于模型代数结果而非实测精度。

当前姿态重新核验相机与全部既有visual AABB分离，中心射线畅通，315条自身遮挡采样左20条、右16条；旧18/20结果只对应前次安装。未提供的支架/线缆、动态全行程及PhysX不在检查范围。

最终USD与前版对比54个机器人网格的points/counts/indices完全一致；只新增显示法线、UV和材质。新增4项CPU USD外观检查已实际运行通过，覆盖平滑/锐边角点法线、掌壳单材质、连续UV及贴图资产。首轮Blender因纹理传输尚未完成显示紫色，未通过；工具现检查依赖存在与SHA并将实际贴图打包。第二轮7张静态图完成，主会话实际查看总体、左右近景及对比图，颜色正确。证据位于私有 `.local/evidence/m5/astra/`，中文图例位于 `.local/evidence/m5/delivery/`。本次没有运行机器人动力学、接触回合、传感器数据或LeRobot导出；生产采集关闭。

集成后执行 `uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v`：93项，85通过、8跳过；其中新增4项因轻量环境无USD依赖而跳过，但已在上述独立CPU USD环境实际4/4通过。日志 `.local/evidence/m5/cpu_tests.txt`。`uv run --frozen --offline --no-python-downloads python -m compileall -q packages scripts tests` 与 `git diff --check` 通过。完整机器命令见 `.local/evidence/m5/astra/COMMANDS.md` 和主会话 `.local/evidence/m5/COMMANDS.md`。

## 2026-09-23 NAS cardbox headless smoke

远端 RTX PRO 6000 查询为约 73.6/97.9 GiB、GPU 100%，没有干预该进程。切换本机 RTX 3080 Laptop（16 GiB，启动前约1.4 GiB占用），复用隔离 venv：Python 3.11.14、Isaac Sim 5.1.0.0、Isaac Lab 0.47.2、Torch 2.7.0+cu128、Isaac Lab commit `3c6e67bb5c7ada942a6d1884ab69338f57596f77`。

直接引用 NAS 官方 `SM_CardBoxA_01.usd` 初次物理/RGB结果 `cardbox01` 证明 480 步可完成，但 UNC MDL 编译失败且图像变红，故不作为材质通过。仅复制已审计的8个文件到私有子集 `cardbox02` 后，材质依赖仍缺失 `OmniUe4Base`，同样不作为最终结果。

随后把该隔离 venv 的 Isaac 资产根设置到 NAS，并保留原扩展设置备份；Windows MDL 对 UNC 模块名编码不兼容，使用同一NAS共享的会话盘符路径。最终 `cardbox04_nas_drive/result.json`：直接从 NAS 盘符加载，`passed=true`，asset dimensions `[0.0839999962,0.0599999973,0.0599999973] m`，uniform scale `0.12`，synthetic mass `0.08 kg`，初始 z `0.2494890541` m，最终 z `-2.05e-8` m，末速度范数约 `1.4e-4 m/s`，RGB `240×320×3`、std `39.6128`。图像 `rgb.png` 实际查看为纸盒棕色材质；该结果只证明合成wrapper下纸盒落体/桌面支撑/RGB和NAS材质解析，不证明机器人、抓取、三路同步或生产质量。

实现入口：`scripts/isaac_smoke.py --asset <NAS-cardbox.usd> --asset-role selected_card_box --synthetic-cardbox-wrapper --graphics-api d3d12 --out <private-dir>`。wrapper明确缩放0.12、质量0.08 kg，惯性由PhysX推导，源USD只读；无该开关仍拒绝无刚体的生产prop。Kit正常关闭曾等待，最终由本次自有 smoke 进程终止并记录；关闭正常返回尚未验收，未用跳过清理来宣称正常关闭通过。

轻量回归：93项，85通过、8跳过；compileall与diff检查通过。生产采集仍关闭。
