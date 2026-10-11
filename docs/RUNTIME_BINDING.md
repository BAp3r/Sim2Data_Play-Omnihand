# 运行时绑定评审

输入是正式发布基线 `0e523e0`、`configs/runtime_candidates.json` 和私有路径绑定。2026-09-22 重新通过已有 SSH 检查资源及三套解释器，没有安装驱动、修改系统包或共享环境。生产运行时仍为 `null`。

## 身份与选择

| 候选 | 重新核对结果 | 决策 |
|---|---|---|
| existing_isaac51 | Python 3.11.15、Isaac Sim 5.1.0.0、Torch 2.7.0+cu128；继承共享环境 | 优先用于有限 smoke，与 NAS 5.1.0 资产配对 |
| existing_isaac50 | Python 3.11.15、Isaac Sim 5.0.0.0；同一份 editable Isaac Lab | 后备候选，不同时启动多个 Kit |
| existing_isaac61 | Python 3.12.13、Isaac Sim 6.1.0.0、Torch 2.11.0；未发现 Isaac Lab | 不作为本轮 Isaac Lab 主线 |

前两套环境解析出的 Isaac Lab 包版本为 `0.47.3`，源码 `VERSION` 为 `2.3.0`。该源码目录没有独立 `.git`，且在外层仓库中是未跟踪副本，因此不能用外层仓库 SHA 冒充上游 Isaac Lab commit。选定的 `__init__.py`、`simulation_context.py`、`simulation_cfg.py` 与官方 `v2.3.0` 同名文件 SHA256 一致；`extension.toml` 不同，本地声明 `0.47.3`，上游 tag 声明 `0.47.2`。这只是选定文件比较，尚未证明整份源码未修改。

官方 `v2.3.0` README 的版本表列出 Isaac Sim 4.5 / 5.0 / 5.1；实际安装身份及实际启动结果仍优先于版本表。上游 tag API 查询遇到限流，未取得可用于绑定的上游 commit。网络错误保留在私有证据中。

`scripts/runtime_identity.py --out <new-private-report.json>` 不导入 Kit，记录解释器、包来源、editable 路径、继承配置和选定源码哈希。报告含本机路径，必须存放 ignored 目录。`uv pip freeze` 对继承式 venv 的结果可能不包含系统 site packages，不能单独作为完整依赖锁。

## 最小验证与后续绑定

`scripts/isaac_smoke.py --asset <reviewed-card-box.usd> --asset-role selected_card_box --out <new-private-directory>` 创建内存场景、合成桌面和一台 320×240 RGB 相机，用 Isaac Lab `SimulationContext` 运行 480 个 1/240 s 步。碰撞和刚体必须来自选定资产；只在开始时设置落体初始位置，步进中不覆写物体位姿。源资产只读，不保存修改回 NAS。输出轨迹、RGB、相机参数和结果 JSON；这不包含双臂、D405 投影标定或接触抓取。

启动前必须核对资产依赖闭包和 GPU 空间，使用离线 uv 复用候选解释器；Kit 扩展在线注册表关闭。每次使用新的私有输出目录，进程由外部超时限制。结果须结合日志与输出图像复核，不能把脚本存在当作通过。

生产绑定还需要：确认整份 Isaac Lab 来源和本地修改；将批准的精确版本与依赖冻结到独立 uv 项目；核验机器人模型及完整装配；完成三路相机投影、碰撞和时序验证。已用官方 Isaac Lab v2.3.0 submodule 和 Isaac Sim 5.1.0.0 解析 `environments/sim/uv.lock`；它是新候选闭包，不是既有继承式共享环境的 freeze，也尚未完整安装/验收。没有为此另装整套 Isaac。

实际执行命令、运行结果及私有证据路径见 `VALIDATION.md`。

## 本轮实际启动结果

第一次使用独立自建 8 cm 立方体（`asset_role=synthetic_fixture`）隔离环境启动，未使用供应商盒子，也未模拟机器人。启动前显存约 8,248 / 97,887 MiB、GPU 利用率 0%。Kit 报告重复的 NVIDIA Vulkan ICD，将同一张显卡枚举为两张，随后 RTX 插件段错误，进程退出 139，尚未进入物理步进和 RGB 输出。不能据此判定 card_box 物理或材质有问题。

日志同时指出默认共享 DerivedDataCache 独占锁失败；仅设置 app/cachePath 并未实现隔离。根据已安装 SimulationApp 源码，启动器已改为明确的 `--portable-root`，并关闭 multi-GPU、只选设备 0。失败时仍保留 `passed=false` 启动标记，不能将崩溃目录当完成产物。上述修改仅编译检查，尚未完成 GPU 复试。

复查 GPU 已被其他任务占用约 81,777 MiB、利用率 100%，本轮停止 GPU 尝试，未停止其他用户进程。下次空闲时可先在单个进程中用 `VK_DRIVER_FILES=<已核对的单一 NVIDIA ICD 文件>` 排除重复枚举，再运行 synthetic fixture 和已审阅 card_box；此处是诊断方案，不是已验证修复。不卸载或升级驱动，不修改系统 ICD、共享环境或全局配置。

已定位的 NAS card_box 尺寸为 0.70×0.50×0.50 m，未 authored rigid body 和质量；当前不能直接作为动态桌面目标传入 smoke。先制定获批准的缩放/动态 wrapper、质量/碰撞和依赖闭包，再测试。详见 `ASSET_AUDIT.md`。
