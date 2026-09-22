# Windows 本地仿真运行环境

本目录描述 Windows x86_64 的独立 Isaac Sim 运行环境。环境目录和下载缓存都在 worktree 的 ignored `.local` 下：解释器为 `.local/envs/sim`，uv 缓存为 `.local/cache/uv`。公开配置是 [`environments/local_windows/pyproject.toml`](../environments/local_windows/pyproject.toml) 和对应的 [`uv.lock`](../environments/local_windows/uv.lock)。

## 已绑定版本

| 组件 | 实际版本或身份 |
| --- | --- |
| Python | 3.11.14，CPython，Windows AMD64 |
| Isaac Sim | 5.1.0.0；NVIDIA Windows `win_amd64` wheels |
| PyTorch | 2.7.0+cu128；PyTorch cu128 index |
| TorchVision | 0.22.0+cu128 |
| Isaac Lab | package 0.47.2；源码 VERSION 2.3.0 |
| Isaac Lab 源码 | 官方 `v2.3.0`，commit `3c6e67bb5c7ada942a6d1884ab69338f57596f77` |
| Sim2Data | `sim2data-core` 和 `sim2data-isaac` 0.1.0 editable path source |

Isaac Lab 的安装入口固定为 `third_party/IsaacLab/source/isaaclab`，不是仓库根目录。Isaac Sim 的 `all,extscache` extras 已包含在锁文件中；锁中 166 个包均按 Windows 标记解析。

Isaac Sim 5.1.0.0 的 Windows wheel 在自身 `METADATA` 中固定了 `filelock==3.13.1`、`fsspec==2024.6.1`、`networkx==3.3` 和 `pywin32==306`。NVIDIA 索引的解析元数据对这四项曾给出不一致的更新约束，因此项目同时保留显式 pins 和 uv `override-dependencies`，以实际 wheel metadata 为准。

## 创建和同步

先确认官方 Isaac Lab submodule 已在固定 gitlink 检出，并使用一个已有的 Python 3.11 解释器作为 venv 的解释器来源。解释器来源只提供标准库和启动文件，不提供旧环境的 site-packages。

```powershell
git submodule update --init --no-recommend-shallow third_party/IsaacLab

$env:UV_CACHE_DIR = (Join-Path (Get-Location) ".local\cache\uv")
uv venv --python <Python-3.11.14-executable> .local\envs\sim

$env:VIRTUAL_ENV = (Resolve-Path ".local\envs\sim").Path
$env:PATH = (Join-Path $env:VIRTUAL_ENV "Scripts") + ";" + $env:PATH
uv sync --project environments/local_windows --active --locked --no-python-downloads
```

使用 `VIRTUAL_ENV` 和 `--active` 是为了把同步目标明确设为 `.local/envs/sim`。不设置这两个选项时，uv 会使用项目目录下的默认 `.venv`。

缓存、临时展开目录、同步日志和身份报告必须保留在 `.local`，不要把它们加入 Git。不要把 Isaac Sim wheels、资产库或模型权重复制到仓库。

## 隔离检查

目标 venv 的 `pyvenv.cfg` 必须包含：

```text
include-system-site-packages = false
```

以下命令不启动 Kit、Isaac Sim 应用或 GPU，只检查锁文件和已安装 metadata：

```powershell
$env:UV_CACHE_DIR = (Join-Path (Get-Location) ".local\cache\uv")
uv lock --project environments/local_windows --check --offline

$env:VIRTUAL_ENV = (Resolve-Path ".local\envs\sim").Path
$env:PATH = (Join-Path $env:VIRTUAL_ENV "Scripts") + ";" + $env:PATH
uv sync --project environments/local_windows --active --locked --check --no-python-downloads --offline
& .local\envs\sim\Scripts\python.exe .local\evidence\runtime_identity.py
```

本次检查结果为锁文件 `Resolved 166 packages`，环境审计 `Audited 165 packages` 且 `Would make no changes`；`uv pip check` 检查 165 个包并报告 `All installed packages are compatible`。Torch/TorchVision 的只读导入报告为 `2.7.0+cu128`、CUDA build `12.8` 和 `0.22.0+cu128`。身份报告记录了解释器、site-packages、Isaac Sim/Torch/Isaac Lab 版本、Isaac Lab commit、VERSION 和关键源码 SHA256。旧共享环境的 `site-packages` 不在目标解释器的搜索路径中；venv 仍会引用创建它的 Python 安装中的基础标准库路径，这是 CPython venv 的正常行为。

## 当前边界

环境子任务只完成依赖解析、安装和身份检查，没有启动 Kit。主会话随后已运行本机 synthetic Cube physics/RGB smoke，实际结果见 `VALIDATION.md`；尚未验证机器人装配、三相机同步、抓取接触或完整数据回合。通过上述 CPU/metadata 检查不代表物理仿真或生产采集验收。

完整命令输出和包含本机路径的身份 JSON 只保存在 worktree 的 `.local/evidence`，不应提交或发布。
