# 数据环境

Linux x86_64 / Python 3.12。`pyproject.toml` 使用本项目 core/lerobot 两个包和 `third_party/lerobot` 官方 submodule；固定源码 commit 见 `docs/PACKAGING.md`。

已实际运行 `uv lock --project environments/data --no-python-downloads`，生成 64 包的 `uv.lock`。这里只做解析，没有安装完整新环境。upstream 的 cu128 index 与显式 Torch 2.11.0+cu128 / TorchVision 0.26.0+cu128 保持一致；格式 smoke 本身只执行 CPU 运算。此前 CPU-only index 尝试与上游 source index 冲突，未保留为配置。

运行环境与 SDK smoke 的实际版本、来源和结果见 `docs/VALIDATION.md`。重建前检出固定 submodule，再在独立数据环境执行 `uv sync --project environments/data --frozen`；不要修改共享环境，也不要在这里加入 Isaac Sim。
