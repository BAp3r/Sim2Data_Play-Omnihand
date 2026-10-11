# 固定上游源码

`IsaacLab/` 和 `lerobot/` 是官方 Git submodule，精确提交由父仓库 gitlink 固定。版本、包版本差异和安装路径见 [PACKAGING](../docs/PACKAGING.md)。使用 `git submodule update --init`，不要用 `--remote` 自动升级。

Isaac Lab v2.3.0（Python 包 0.47.2）和 LeRobot 0.6.2 的许可证保留在各自源码内；本项目 MIT 许可不替代上游许可。第三方资产全集、权重、环境和数据集仍留外部私有存储。

两套独立 uv 环境在 `environments/sim` 和 `environments/data`；已生成真实锁文件，完整环境安装及生产验收尚未完成。
