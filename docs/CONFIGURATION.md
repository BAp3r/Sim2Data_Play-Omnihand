# 本机与服务器还需要配置什么

`assets/` 和 `third_party/` 只解决文件放在哪里。实际采集还需要路径绑定、独立环境、机器人/标定、任务/数据契约和写入目录。下列“待配置”均不是已完成的运行时绑定。

| 配置项 | 位置或入口 | 当前状态与用途 |
|---|---|---|
| 机器路径与 NAS 挂载 | ignored `.local/paths.json`；公开说明 `configs/paths.example.json` | 已记录部分私有盘点路径，生产数据根和运行时未绑定。Windows UNC 与 Linux 挂载分别填写；符号链接不能代替网络挂载 |
| 仿真环境 | `configs/runtime_candidates.json`、`scripts/runtime_identity.py`、`docs/RUNTIME_BINDING.md` | 三套候选已复核；5.1 synthetic 启动失败，生产绑定仍为空。`environments/sim/uv.lock` 已实际解析；未完整安装或验收，不把共享环境 freeze 当闭包 |
| 导出环境 | `environments/data/pyproject.toml` + `uv.lock` | 与仿真环境隔离；固定官方 LeRobot SDK/视频编码依赖。根 `uv.lock` 仅覆盖无重依赖的 M0 工具 |
| 机器人与资产 manifest | `configs/asset_manifest.json` | 官方来源/版本/hash/许可、arm 与 hand 的真实驱动映射、单位和限位；已有静态审计 manifest，尚未生产绑定 |
| 标定与装配 | 原件放 `.local/calibration/` 或 ignored `calibration/`；`configs/assembly_inputs.template.json`、`docs/ASSEMBLY_INPUTS.md` | 已提供逐侧原子链和安装刚体输入模板；它不是可直接合并的场景配置。实测值仍为空；主相机位姿及覆盖也必须核查 |
| 任务与采集 schema | `configs/scene_spec.draft.json` | 桌面中转、三路 RGB 和显式门禁已定义；阈值、可达性、反馈/命令维度尚未冻结，depth 默认关闭 |
| 活动数据与缓存 | ignored `outputs/`、`datasets/`、`.cache/`、`logs/`、`reports/` | 实际可写根目录放服务器本地盘；每 writer 独立临时根，成功回读后归档 NAS。不要把不同进程指向同一个数据根 |
| uv / Kit 缓存 | 用户级 `UV_CACHE_DIR` 与未来启动器配置的 Kit cache | 保留已有缓存；uv cache/venv 尽量同文件系统以便 hardlink。Kit 缓存只在实际启动器中绑定，不能假定通用环境变量会生效 |
| Git 认证、分支与 LFS | 两个既有 remote、`.gitattributes`、本机凭据存储 | 向同名审阅分支推送代码。原有 main 与 LICENSE 保留，不 force push。真实 LFS 对象尚需单独上传/干净拉取验证 |
| Agent 模板 | `AGENTS.md`、`.codex/config.toml.example`、`.codex/agents/` | 仅为项目约定/模板；先审阅合并，不覆盖用户配置，不把模板当启动记录 |

`SIM2DATA_ASSET_ROOT`、`SIM2DATA_ROBOT_ROOT`、`SIM2DATA_DATA_ROOT` 是规划中的路径接口。当前 M0 工具按其命令行参数工作，尚未实现统一读取 `.local/paths.json` 或自动加载 `.env` 的运行器；仅创建目录或设置变量不会启动采集。`production_collection_enabled` 保持 false。

## 公开与私有存储

此前两个远端已同步 M0 公开安全快照；本轮实施提交尚未推送。NAS 有足够容量也不表示所有文件都应进 Git LFS：官方全集留外部，数据集/实录/标定放私有文件存储；自建或获授权的 USD 才进入版本库。若未来内网 Git 需要包含私有资产，使用独立私有资产仓库或专门发布流程，避免把带私有历史的分支直接推到公有 GitHub。

`third_party/IsaacLab` 与 `third_party/lerobot` 已作为官方固定提交的 Git submodule 纳入；其他外部内容继续忽略。版本和独立环境见 `PACKAGING.md`。`.gitignore` 不会自动取消已跟踪文件，也不是保密审计。每次公开发布只选已审阅文件，不推送临时 worktree 中尚未完成的 A/D/E 内容。

## 配置顺序

1. 固定两个远端的认证方式和审阅分支，保持 HTTPS 证书校验开启。
2. 确认本地盘数据/缓存目录与 NAS 的真实只读资产路径。
3. 绑定机器人、盒子资产和标定输入，核定单位与坐标链。
4. 使用已生成的独立仿真/导出候选锁完成隔离安装与运行时兼容性验证，再冻结生产绑定。
5. 通过接触、相机与官方 SDK 验收后，再配置批次、seed、数据划分、并行度和归档策略。

## 本轮运行时证据与下一次启动

主会话的实际命令及机器路径集中在 ignored `.local/evidence/m1/COMMANDS.md`；`runtime51.json`、`runtime50.json`、`runtime61.json` 分别记录候选解释器和源文件身份。公开配置只使用候选 ID，不含服务器绝对路径。源文件 SHA256 只证明被比较的文件，不能替代缺失的上游 commit 或完整来源审核。

`isaac_smoke.py` 使用新的输出目录，源 USD 只读；其 `--asset-role` 必须声明 synthetic fixture 或 selected card_box。第一次 synthetic Kit 启动退出 139，后续 portable-root/单 GPU 设置只完成编译检查。下一次启动先确认 GPU 已空闲并选择单一 Vulkan ICD，再审查日志实际使用的 cache/data/log 路径。禁止通过修改共享缓存、驱动或系统 ICD 排除故障。

装配工具接受已知原子 `FrameTransform`，不会从名义 4 cm 或视觉推断生产 SE(3)。每侧安装件和相机载荷分别记录，合并刚体前确认参考 frame、质心和惯性；depth 继续关闭。批准拓扑观察包原件与视频哈希在 ignored 私有证据目录，公开文件只引用 observation ID。
