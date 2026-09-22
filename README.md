# Sim2Data — 双臂桌面中转数采框架

2026-09-22。AIRBOT Play + OmniHand 2025：左臂从桌面左侧取盒，桌面中转松手，右臂重新抓起并放入桌外右侧框子。两路腕部 D405 和一路通用主相机；目标格式为 **LeRobotDataset v3.0**。

## 交付边界

当前为 M0/M1 前置框架，**尚未完成物理接触抓取或正式机器人数据采集**。设计、输入参数和实施顺序见 [实施计划](docs/EXECUTION_PLAN.md)。远程发布采用基于既有 GitHub main 的独立审阅分支，保留原始历史与 LICENSE；开发 worktree 中未完成内容不随发布快照提交。

设计、计划和 [代理协作约定](AGENTS.md) 已交付。用户已恢复资产/模型审计、装配输入、运行时验证与官方 SDK synthetic smoke；旧在途工作保留并经审阅复用。接触任务和正式批量采集继续受关卡阻断。

已实现：依赖为零的主机/资产/视频文件盘点、可重复的 episode/component seed、资产路径和 LFS 指针检查、具名 state/action 维度检查、记录时钟同步契约，以及对应单元测试。

已实际访问本机实拍、NAS 和 GPU 服务器，复用现有解释器盘点了运行时版本；三个 Luna Max 第一波子代理已启动。机器路径、实拍派生图和完整原始报告仅放 ignored 私有目录。最新验证范围见 [验证记录](docs/VALIDATION.md)。

仍待验收：匹配实物版本的机器人 articulation、完整装配/标定、真实接触接力、生产 LeRobot 回合、GPU 渲染性能及远程 LFS 干净拉取。

## 先读

- `docs/DESIGN.md`：架构、控制与数据契约、验收关卡。
- `docs/EXECUTION_PLAN.md`：本次确认的桌面接力目标和依赖顺序。
- `docs/CONFIGURATION.md`：除资产/第三方目录外，路径、uv 环境、标定、数据写入和 Git 还需配置什么。
- `docs/SCENE_REVIEW.md`：视频证据、假设和缺失测量。
- `.local/DEPLOYMENT.md`：本次项目的私有路径与两台机器的盘点命令。此目录已被 Git 忽略。
- `AGENTS.md` 与 `tasks/`：Luna Max 子任务边界、交付格式和执行顺序。
- `configs/scene_spec.draft.json`：未标定参数为 null，禁止作为生产场景配置。
- `configs/runtime_candidates.json` 与 `docs/RUNTIME_BINDING.md`：候选身份、运行时验证状态与剩余绑定条件。

## 无重型依赖测试

已有 Python >= 3.10 与 uv 时，在此目录执行：

```bash
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
```

只读预检：

```bash
uv run --frozen --offline --no-python-downloads python -m sim2data.preflight
```

当前预检返回 **2**，列出缺失参数与固定设计契约冲突并阻断采集，这是预期结果。M0 预检不校验全部数值或物理有效性，即使手工填满参数也不授权采集；物理/场景/正式数据证据仍须通过后续关卡。加 `--out reports/preflight.json` 可保存新报告，已有文件不会被覆盖。

`uv.lock` 仅锁定这个无外部依赖的 M0 项目，**没有锁定 Isaac Sim / Isaac Lab / LeRobot**。仿真和导出环境的 Python、CUDA/PyTorch 和软件版本，需要在实际服务器盘点后分别锁定。

## 单文件盘点入口

`collect_inventory.py` 自身不依赖本项目其他模块，可以单独拷贝运行：

```bash
uv run --no-project --offline --no-python-downloads python scripts/collect_inventory.py --out reports/host.json
```

可增加 `--asset-root`（可重复）和 `--video-root`。Windows 支持本机可访问的 UNC 路径；Linux 必须传已挂载的路径，不能把 Windows UNC 直接当 Linux 挂载。

脚本只写指定报告，不安装软件、不运行仿真、不下载或复制资产、不调用机器人、不读取私钥、不读取 Git 远程 URL。报告可能含本机路径和网络挂载名，应保持私有。默认每个目录最多扫描 10 万文件、45 秒、保留 200 个候选匹配；达到限制返回码 2，并标记不完整，不能据此断言资产不存在。NAS 阻塞的单次系统调用可能超出协作式时间预算。

“候选文件”只按路径名称匹配，不等于已验证有效的 USD、动力学模型或合规授权。下一阶段必须验证真实资产及依赖闭包。

## 文件用途

`reference/` 只有少量上传视频的派生关键帧及接触表，不包含原始视频。它和 `.local/`、`reports/`、外部资产、数据集均被 Git 忽略。`.gitattributes` 已配置 USD/USDA/USDC/USDZ 等模型文件走 LFS；安装并验证 Git LFS 后才可提交真实模型。请保留目标仓库已有的 LICENSE，第三方资产不因项目代码许可而自动获得再分发许可。
