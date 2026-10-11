# 仿真环境

Linux x86_64 / Python 3.11。Isaac Lab v2.3.0 的发行包版本为 0.47.2，path source 是 `third_party/IsaacLab/source/isaaclab`；搭配 Isaac Sim 5.1.0.0 与 Torch 2.7.0+cu128。submodule 的固定官方提交见 `docs/PACKAGING.md`。

此声明与服务器的未版本化 0.47.3 editable 副本区分记录。本轮不升级/修改共享环境。`uv lock` 仅负责解析，完整新仿真环境的 sync 和生产绑定未完成；首次 Windows 解析遇到 NVIDIA 大 wheel 的范围读取/内存映射错误，随后在隔离 Linux 工作目录成功解析 200 包，并通过离线 `uv lock --check`。Windows 离线复核缺少对应元数据缓存，未通过；完整记录见 `docs/VALIDATION.md`。

生产绑定还需处理已观察到的 Vulkan/RTX 启动失败、核对资源与资产依赖，然后执行 physics/RGB 和双臂装配验证。目录或锁文件不能代替这些验收。
