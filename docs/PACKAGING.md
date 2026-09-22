# 三包源码与独立环境

本轮按用户指定目录完成源码迁移。`packages/sim2data_core` 保存数据契约、配置预检和资产清单接口；`packages/sim2data_isaac` 保存坐标链与装配需求；`packages/sim2data_lerobot` 保存官方 SDK writer/loader 适配器。接触控制和 Mimic 运行时适配仍待 A/B 关卡，目录存在不表示这些功能已完成。

三个发行包分别为 `sim2data-core`、`sim2data-isaac`、`sim2data-lerobot`。保留 `sim2data.core`、`sim2data.assets`、`sim2data.backends.isaaclab`、`sim2data.export` 导入接口。仅 core 拥有顶层 `sim2data/__init__.py`，用 `pkgutil.extend_path` 扩展路径；另外两包提供 namespace 子目录，避免重复安装同一个 initializer 文件。实际 editable 安装和跨包导入已检查。

根 `pyproject.toml` 是 CPU 开发 workspace，`uv.lock` 只含本项目的四个项目记录，不含 Isaac、Torch、LeRobot。首次构建需要小型 setuptools 构建工具；构建缓存就绪后可以离线运行契约测试：

```powershell
uv run --frozen --offline --no-python-downloads python -m unittest discover -s tests -v
uv run --frozen --offline --no-python-downloads python -m sim2data.preflight --scene configs/scene_spec.draft.json
```

预检默认场景路径仍相对于工作目录，外部调用应显式传 `--scene`。`load_manifest()` 会沿 editable 源码的父目录寻找仓库公开 manifest；纯 wheel 安装而没有仓库时必须传显式路径，不把机器配置打入 wheel。

## 上游源码

`third_party/IsaacLab` 与 `third_party/lerobot` 是真实 Git submodule，`.gitmodules` 保存官方 HTTPS URL，Git gitlink 固定精确提交：

| 上游 | 固定提交 | 源码身份 |
|---|---|---|
| Isaac Lab | `3c6e67bb5c7ada942a6d1884ab69338f57596f77` | 官方 tag v2.3.0，Python 包 extension 版本 0.47.2 |
| LeRobot | `b64fe1ed9f11eeac53ee821356d2797601701054` | 官方可取回 commit，项目版本 0.6.2，dataset 格式 v3.0 |

服务器此前 LeRobot 0.5.2 checkout 的 `377e82…` 无法从官方远端 fetch，不能把它作为官方来源绑定。服务器既有 Isaac Lab 0.47.3 副本也不等于本 submodule 的 0.47.2。现有共享环境只读保留，源码 submodule 的建立不自动升级它们。

新检出使用 `git submodule update --init`，不使用 `--remote` 自动追分支；无需递归下载上游的可选子模块或大型 LFS 对象。两个框架源码许可分别保留在其 submodule 中，不将第三方资产库、模型权重或运行环境嵌入父仓库。

## 两套运行环境

`environments/sim` 与 `environments/data` 不属于根 CPU workspace 的成员，各自有独立项目与锁，目标均为 Linux x86_64：

- sim：Python 3.11，Isaac Sim 5.1.0.0，Torch 2.7.0+cu128，Isaac Lab path 指向 `third_party/IsaacLab/source/isaaclab`，不是没有 Python 项目的上游仓库根。
- data：Python 3.12，LeRobot 0.6.2 path source，Torch 2.11.0+cu128 / TorchVision 0.26.0+cu128。官方源码的 uv 配置选择 cu128 index；导出测试仅执行 CPU 运算，这不表示 wheel 本身是 CPU-only 版。

锁文件必须来自实际 `uv lock --project environments/<name>`。数据环境已解析 64 包、仿真环境在隔离 Linux 工作目录解析 200 包生成锁；锁的生成不安装这些重依赖，也不证明锁定环境已经运行。仿真锁解析及 SDK/物理实际执行的最终结果见 `VALIDATION.md`。不要将候选锁的解析通过与 M1/M4 验收混为一项。

需要新环境时在独立目录 `uv sync --project environments/<name> --frozen`；本轮优先复用服务器已有依赖进行受控验证，没有执行这两个完整运行环境的 sync。生产运行时和采集开关继续关闭。
