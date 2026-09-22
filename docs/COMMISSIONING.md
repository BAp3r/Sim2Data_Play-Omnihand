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

用户要求新的 Astra xhigh/max 装配复核；前两次 spawn 被线程上限拒绝，模型审计任务完成释放名额后，GPT-6 Astra Max 已实际启动。它已发现初版转接件悬空和相机掌宽侧置问题，正在量化修正。先前 High 的 CAD 观察与本次 Max 复核分别记证，尚未批准的安装值维持待审。


## 真实几何静态预览与 D405 来源

`MODEL_BINDING.md` 记录新取回的官方 O10 左右模型：各 18 个被引用 STL，17 个 revolute 其中 6 个 mimic，另 middle_abad 为上下限/effort/velocity 都为零的锁定候选；左名保留上游拼写 `l_middle_abad_jonit`。它不代表 11 个电机，也不替代旧私有快照。当前未冻结 action 映射。

旧 AIRBOT 候选 URDF 的 mesh 包没有闭合。为让几何可审阅，单独取同一 DISCOVERSE commit 的 `new_airbot_play` MJCF 与对应 11 个 OBJ，按原 body transform/q=0 生成 **固定关节 visual snapshot**，去除旧手指和 camera_stand。它不是旧 URDF 的补丁、不是动力学转换。私有转换脚本/输入身份在 `.local/evidence/m2/prepare_visual_binding.py`、`assembly_inputs/arm_pose/source_identity.json`。左右组合 URDF 实际生成于 `combined01/`；49 个唯一 mesh 输入生成并重新打开静态 USD，196 个 prim、3 个 Camera。CAD 与第三方派生 USD/BLEND 均不提交。

D405 来源为官方 RealSense ROS 仓库 commit `9a11121700cb4780e273e34141f6402fe184321d`：`realsense2_description/urdf/_d405.urdf.xacro` 与 `meshes/d405.stl`，证据 `.local/evidence/m2/d405/source_identity.json`。STL 数值为毫米，按上游 `scale=0.001` 转米，不能二次缩放。选 `camera_housing` 为官方 bottom screw frame，组合名义内部链为：

- screw→visual mesh：xyz `[0.01465,0,0.021]` m，rpy `[π/2,0,π/2]`；
- screw→color optical：xyz `[0.01085,0.009,0.021]` m，rpy `[-π/2,0,-π/2]`。

这只是固定版本源码中的 nominal extrinsics，不是每台实物标定。源码明确说明 inertial 不可靠、不应用于建模，因此未把其质量/惯量填为生产输入。外部 mount→screw 安装关系仍独立审阅。

本机 Blender 5.2.2 LTS 已通过 `scripts/blender_review_scene.py` 真实导入初版 USD，CPU Cycles 生成3张640×480静态图和私有 `.blend`，记录在 `.local/evidence/m2/blender_review01/`。这些不是 Isaac 相机帧或物理回合。初版 displayColor 在 Blender 未成为有效表面，已给 USD 工具补显式 PreviewSurface 材质绑定，待下一版复核。

## 后续修正

ROS2 完整包已闭合并用于 `combined02` / `assembly_preview02`，此前 missing mesh 仅适用于旧单文件检查。Astra Max v2 将左/右 housing 的 mount Y 设为 +0.011/-0.029 m，固定网格姿态采样自遮挡降至17/315和20/315；该姿态左拇指零值略超限，不能视为有效关节态或覆盖验收。link6夹爪座残留及法兰接合缺口未解决。Blender review02 已用显式 PreviewSurface 完成第二次实际静态渲染。

新 uv 环境第一次运行完成 Kit 启动，但 smoke 错设 `ISAAC_LAUNCHED_FROM_TERMINAL=True` 跳过物理上下文初始化。已移除覆盖，保留 SimulationApp 生命周期设置，继续新目录复试。退出码0不作为通过证据，必须读取 result.json。

## 本机 headless 物理/RGB 结果（2026-09-22）

新隔离 Windows 环境（Isaac Sim 5.1.0.0、IsaacLab 0.47.2/v2.3.0、Torch 2.7.0+cu128）已实际完成 synthetic 8 cm Cube 的480步测试，dt=1/240 s、CPU PhysX + D3D12 RTX RGB。中心高度从0.289489 m降至0.03999999 m，末速度约0.000176 m/s，RGB为240×320×3，标准差39.2979；主会话已查看实际图像。`newenv_smoke02/result.json` 为 passed=true / completed。该结果仅证明桌面夹具下落支撑和单相机渲染，不包含官方 card_box、机械臂、三相机同步或接力数据。Kit 首次着色器编译较慢，物理结果写完后关闭阶段另行记录，不以退出码替代结果检查。

精确机器命令与私有证据见 `.local/evidence/m2/COMMANDS.md`。装配复核见 `ASSEMBLY_VISUAL_REVIEW.md`；生产开关保持关闭。

关闭阶段补充：结果写完后 Kit 超过4分钟未退出，主会话核对进程命令后仅终止本次 smoke。物理/RGB通过，正常关闭未通过；证据 `.local/evidence/m2/newenv_smoke02_shutdown.json`。Blender review04 改用独立灯光/曝光，已实际查看；它对应 camera v1 位置的 preview02，不能用作公开 v2 偏置的复验。

## 用户单臂包修正（当前选择）

前述ROS2/DISCOVERSE夹爪残留候选已被用户提供的 `play` 包替代，当前配置使用该URDF的SHA256校验。6轴链保留，按新坐标重新定义synthetic安装变换；旧-51.4mm显示偏置不再使用。桌面绑定官方Thor，参见 `ASSEMBLY_CORRECTION.md`、`THOR_TABLE_AUDIT.md`。本次新的静态USD/Blender近景不是新的物理smoke；证据目录名m3也不表示M3接触验收。

## 当前相机对称与外观版本

`WRIST_SYMMETRY_REVIEW.md` 取代旧版仅彩色光心对称的安装候选，右手仍使用独立OmniHandright4/R_palm。`APPEARANCE_REFERENCE.md` 描述官网参考配色和局限；物理安装、支架及右optical滚转的真实标定尚待验证。正面/俯视比较图属于审阅相机，不是三路采集数据。

## 实录外观和相机外移增量

当前synthetic profile继续使用用户单臂、独立左右OmniHand和官方D405。按用户要求，左右相机从前次对称候选各向场景外侧增加20 mm；其安装旋转、内部光学链与源几何不变。掌壳银白材质与臂侧平滑着色参考用户实录；静态显示修正不解决支架、承力、驱动约束、标定或接触验收。生产配置保持未标定且采集关闭。
