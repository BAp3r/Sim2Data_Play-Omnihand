# Synthetic 装配调试（M2 准备）

2026-09-22 用户授权 synthetic 桌面、盒子和安装假设，与实测配置隔离；授权及时推送实施分支。`configs/commissioning.synthetic.json` 是调试设计，**不是标定结果或已通过的装配**。原 `scene_spec.draft.json` 与装配实测模板的 null 不被覆盖。

## 已实现接口

- `commissioning.py`：逐侧读取明确 arm/hand URDF、flange/hand root、package roots 和四条原子变换；保留法兰以上链、删除旧末端、添加转接件/相机壳体/光学 frame；重命名 joint/link/material/mimic，验证图和资源闭包。输出 URDF 含私有绝对 mesh 路径，仅存私有目录。
- `scripts/assemble_commissioning.py`：消费公开 synthetic profile 和私有模型 binding JSON，生成左右组合 URDF 与摘要。CAD mesh 可作为 mount visual，质量/碰撞仍需另行核定。
- `scripts/build_commissioning_preview.py`：CPU USD 静态预览，真实 visual mesh、独立关节零位及 URDF mimic 的 FK、桌面/中转标记/框子/盒子和三个 Camera prim；导出并重新打开检查。它没有 articulation/drive/collision cooking，不可导出为采集回合。

```text
uv run --frozen --offline --no-python-downloads python scripts/assemble_commissioning.py --profile configs/commissioning.synthetic.json --binding <private-binding.json> --out <new-private-urdf-dir>
uv run --no-project --offline --no-python-downloads --python <isolated-pxr-python> python scripts/build_commissioning_preview.py --profile configs/commissioning.synthetic.json --left <left-combined.urdf> --right <right-combined.urdf> --out <new-private-preview-dir>
```

binding 对每侧包含 `arm_urdf`、`hand_urdf`、`flange_link`、`hand_root_link`、`package_roots`，以及可选 `mount_visual={filename,origin:{xyz_m,rpy_rad}}`、`mount_mesh_base`。profile 的 wrist housing 是明确标记的几何 proxy，不是 D405 厂家模型。左右解剖型是调试选择，不是实物识别。

## 运行时进度

远端检查仍为 GPU 100%、约 81.8 GB 占用，没有启动新的远端 Kit。用户允许本机 headless；发现已有 Isaac Sim 5.1 / Torch 2.7 / Isaac Lab 0.47.6 环境，保留不修改。首轮本机 Vulkan 在 RTX 场景插件初始化崩溃；单独 `--d3d12` 仍实际选择 Vulkan。核对 .kit 源码后同时覆盖 `--/app/vulkan=false`，日志确认 headless 启动完成，随后在源 Cube 被强制覆盖成 Xform 导致空 bounds 的脚本检查处退出。已修复引用 prim 的类型覆盖，尚需新隔离环境复试，不能宣称 physics/RGB 通过。

用户随后要求子代理用 uv 建立独立环境，后续测试切换到其独立 worktree 环境，不再继续旧环境尝试。原有环境、驱动和系统设置均保留。

## 验证与边界

URDF composer 的 7 项 CPU 测试通过；覆盖旧末端裁剪、mimic重命名/悬空拒绝、mesh路径逃逸/缺失、显式transform、可选CAD/housing及禁止覆盖。USD工具在双侧简单几何夹具上实际生成并重新打开，确认3个Camera prim；该夹具不是机器人装配验收。第一次运行发现NumPy float32与Gf构造接口不兼容，修复为Python float后通过。

详细本机日志和身份保存在 `.local/evidence/m2/local_runtime.json`、`local_smoke01*`、`local_smoke02_d3d12*`、`local_smoke03_d3d12_override*`。本轮尚未生成新的物理采集回合。A/B输入、关节动力学、必要碰撞及三相机视场通过前，完整接力仍不可称成功。

用户要求新的 Astra xhigh/max 装配复核；模型清单支持，但本聊天 spawn 两次被线程上限拒绝。先前 Astra High 的 CAD 观察仍有效，不能将其称为这次 Max 复核。尚未通过 Max 复核的 synthetic 安装值维持待审状态。
