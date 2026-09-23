# 本机与服务器还需要配置什么

2026-09-23 已新增 `scripts/isaac_scene_smoke.py` 作为私有静态运行时入口。它只接受已审阅的 assembly/profile、manifest 对齐的 Thor/cardbox 路径，使用 session layer 绑定 NAS，不写源 USD；输出三路 RGB 到 ignored `.local/evidence/m7/`。该入口不绑定生产机器人 articulation、驱动、标定或数据根，`production_collection_allowed` 永远为 false。

可选 `--trajectory <Pinocchio-plan.json>` 进入用户授权的运动学回放采集，输出 `capture.json` 和三路 PNG；`scripts/export_motion_sample.py --capture-dir <capture> --output-root <new-dataset> --report <private-report>` 用官方 SDK 写入/回读。准确机器命令在 `.local/evidence/m8/COMMANDS.md`。其 12 维规划配置 schema 只属于这条 synthetic 样本，不冻结生产 state/action 维度。

`assets/` 和 `third_party/` 只解决文件放在哪里。实际采集还需要路径绑定、独立环境、机器人/标定、任务/数据契约和写入目录。下列“待配置”均不是已完成的运行时绑定。

| 配置项 | 位置或入口 | 当前状态与用途 |
|---|---|---|
| 机器路径与 NAS 挂载 | ignored `.local/paths.json`；公开说明 `configs/paths.example.json` | 已记录部分私有盘点路径，生产数据根和运行时未绑定。Windows UNC 与 Linux 挂载分别填写；符号链接不能代替网络挂载 |
| 仿真环境 | `configs/runtime_candidates.json`、`scripts/runtime_identity.py`、`docs/LOCAL_RUNTIME_ENV.md` | 远端三套候选保留历史结果；本机 Windows 隔离 5.1 已安装并完成单盒物理/RGB、完整装配静态三路 RGB，正常关闭和生产绑定仍未通过。Linux sim 锁不代表已安装 |
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

两个远端已同步实施分支的已审阅提交；实际发布 SHA 以交付及私有发布记录为准。NAS 有足够容量也不表示所有文件都应进 Git LFS：官方全集留外部，数据集/实录/标定放私有文件存储；自建或获授权的 USD 才进入版本库。若未来内网 Git 需要包含私有资产，使用独立私有资产仓库或专门发布流程，避免把带私有历史的分支直接推到公有 GitHub。

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

## 本机隔离调试配置

`scripts/assemble_commissioning.py --profile configs/commissioning.synthetic.json --binding <private-binding.json> --out <new-directory>` 接受逐侧模型包与可选 CAD/D405 visual。私有 binding 不提交。D405 STL 按上游 scale=0.001 转米；housing 为 bottom screw frame。synthetic v2 左右 housing 横向偏置为设计选择，非实测。`isaac_smoke.py --graphics-api d3d12` 同时设置 Kit app.vulkan=false，仅作用当前进程。Blender 工具仅生成静态审阅图，不作为训练 RGB。

新synthetic profile的 `arm_model.urdf_sha256` 绑定用户 `play` 包；组合工具拒绝旧arm文件，防止错误复用安装变换。`robots.*.preview_joint_positions` 仅控制静态FK显示，缺省取限位内最接近零并应用mimic。`table.asset_id/source_meters_per_unit/source_up_axis/T_world_asset` 与私有 `--table-usd` 共同绑定Thor桌；无源绑定会失败，不回退方块桌。`--wrist-closeups` 只增加审阅视角，不增加训练通道。

逐侧 `robots.*.hand_model.urdf_sha256` 防止误绑手侧。`camera_symmetry` 明确中心对称、右optical滚转π、无负scale/无像素翻转；`T_mount_camera_housing` 右侧为 `[.038,-.031,.012]`、RPY `[π,-π/2,0]`。配色入口为 `configs/appearance.reference.json` 和预览 `--appearance`，Blender `--material-mode reference --symmetry-views` 保留参考材质并增加审阅图，三路训练通道不增加。

用户追加的外移20 mm仅作用synthetic安装：左housing从mount Y=+.011改为+.031 m，右从-.011改为-.031 m，X/Z及RPY保持不变。在当前预览姿态中分别约对应world X=-.020/+.020 m的增量（URDF近似角造成微小离轴分量）；它不写入生产标定。

`scripts/isaac_smoke.py --asset <reviewed-cardbox.usd> --asset-role selected_card_box --synthetic-cardbox-wrapper --graphics-api d3d12 --out <new-private-directory>` 在新内存场景中加入0.12倍缩放、0.08 kg刚体wrapper；保留源碰撞。没有该显式选项时仍要求输入自身已有单刚体。素材及七个显式依赖可按已审manifest哈希建立私有本地子集，以验证Windows UNC材质解析问题；不复制资产全集。输出不进入生产数据集。模型模板默认改为 `gpt-6-luna`，专项Sol使用 `gpt-6-sol`，不覆盖用户配置或改写历史执行记录。

2026-09-23 用户要求独立venv默认资产指向完整NAS。实际修改该环境 `isaacsim.storage.native/config/extension.toml` 的 `/persistent/isaac/asset_root/default`，先备份再原子替换以避免修改uv缓存硬链接。私有 `<venv>/sim2data_nas.json` 保存asset_root和MDL搜索目录；smoke在启动Kit前读取它并设置进程级MDL_SYSTEM_PATH/MDL_USER_PATH以及明确的Kit资产根设置。Windows UNC在MDL模块编码中失败时使用同一NAS共享的会话盘符，保留盘符而不resolve回UNC。该设置不改源资产或全局环境；uv重装包可能覆盖，需从私有profile重新应用。机器路径/盘符和还原备份仅存私有证据。


## 2026-09-23 合成夹爪 commissioning 配置

`configs/commissioning.synthetic.json.gripper_commissioning` 为左右独立开合映射。`amount=0` 是张开，`amount=1` 是闭合；每侧 10 个 active joint 目标按各自 URDF limit 限幅，mimic joint 由模型耦合。`synthetic_drive` 的 stiffness/damping/max_force/max_velocity_rad_s 是明确假设，不覆盖 manifest 中生产参数的 null。

当前固定手型候选采用内网 teleop 仓库已实现的 `tripod` 端点；来源 commit 与左右十维端点写入 profile。该代码的硬件发送仍展开为十个主动关节，不能把一维接口误解为十个关节共用同一角度。

框子顶面与桌面平齐是 synthetic 场景约束，待官方资产/尺寸绑定后再验证，当前不会写入生产标定。

场景几何已改为连续的 `/World/FullFlatGround` 平面；不再用大方块冒充地面。Thor 桌面仍通过官方 table USD 引用，外侧框子 rim 的 synthetic `top_z_m=0` 与桌面基准平齐，底面高度由配置显式给出。
