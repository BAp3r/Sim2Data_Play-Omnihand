# 全局状态规划、Thor 场景与候选资产（2026-09-24）

全局状态能够提供物体位置、机器人关节状态和几何，从而求解接触候选与机械臂 IK；有 IK 解仍需检查手指包络、运动路径、动力学跟踪及接触保持。

`scripts/plan_contact_trajectory.py` 消费真实组合 URDF、synthetic profile 和 asset manifest。它读取源 mimic 关系、采样源 STL 碰撞表面，以 SciPy 最小二乘拟合对向指尖位置并求六轴 IK。输出逐候选残差、碰撞净空与路径门禁，失败保持 `execution_allowed=false`。这不是 PhysX 控制器，也不生成抓取视频。

M12 `planner_left10` 的左侧候选 `top_down_x_roll_negative` 接触拟合和 IK 收敛，IK 数值残差约 `1e-16 m/rad`，但预抓取 index DIP 约穿盒 7.2 mm，抓取端点也有非预期穿透，不能执行。右侧 `planner_right02` 同样失败。当前固定闭合手型、有限方向搜索和采样几何不构成完备运动规划；未求解连续三角形碰撞、自碰撞、相机支架碰撞或摩擦锥稳定性。

场景在 session layer 中引用官方 Thor table 和官方 NVIDIA cardbox，并核对源 SHA。连续 `/World/FullFlatGround` 是带 CollisionAPI 的 Mesh；cardbox 均匀缩放 `0.12`，几何尺寸约 `84 × 60 × 60 mm`；动态刚体初始位置只在仿真前设置，保留重力与源 boundingCube 碰撞。框子底板及四壁带碰撞，框顶与视觉桌面平齐。Thor 视觉顶面 `z=0` 与碰撞顶面 `z=-0.0155 m` 分开记录。质量、摩擦、装配、驱动及位置均是 synthetic。

M12 `scene05` 已恢复机器人渲染：session 内解除实例并逐 render mesh 标注语义，主视角 robot/cardbox 像素为 `7594/537`，近景为 `36810/7419`。图像能证明场景可见性，不能给出接触力或抓取成功。Kit renderer warmup 的 app updates 会推进少量仿真时间，不能把旧报告中的 `physics_steps=0` 理解成严格无物理推进；显式步进与预热在新版报告中分开。`scene06` 暂停时间线的诊断未取得有效 RGB，保留失败记录。正常关闭仍未通过，进程退出单独记证。

官网仿真页链接 ModelScope `RoboFlywheel/Rigid`。固定 revision `b288a2e15ee585a7d1c7a756e20d4c37c2817249` 的 README 声明 Apache-2.0，但底层资产来源未独立核验。两个盒子候选的 20 个文件（74,060,878 bytes）已写入 NAS 独立 Candidates 目录并逐文件核 SHA。公开仓库仅保存 `configs/asset_candidates.roboflywheel.json`，没有资产副本或机器根路径。

| 候选 | 实测尺寸 (m) | 质量 (kg) | 碰撞 hull | 结论 |
| --- | --- | --- | --- | --- |
| `27_26_0000` Parcel Box | 0.200 × 0.068779 × 0.067784 | 0.1298，mass_check=ok | 4 | 优先保留候选，长边大于当前盒子，不自动替换 |
| `6_30_0000` Folding Carton Box | 0.122126 × 0.031987 × 0.150 | 0.05，mass_check=flag | 1 | 暂不优先采用 |

两者均未做 PhysX 接触运行验收，主场景仍使用原官方 NVIDIA cardbox。生产采集、成功抓取与双臂接力验收继续关闭。

空载响应：left_response05 fresh USD 实测 1920 PhysX 步、real_articulation=true、finger_target_response=true，生成两路 RGB；没有物体因此 contact=false。Kit close 仍未正常返回。

最终修正拟合：使用盒面内缩 patch 的实际距离，避免带方向残差相互抵消，同时惩罚非接触手链路穿盒。补查开手到位端点和起始构型，runtime 将起点读回与计划逐关节比较；不匹配则阻断，不写关节状态。最新左候选仍有 77 个路径采样失败；右 planner_right03 接触拟合和碰撞失败，exit 3，均不执行。8 项规划器专用测试通过。
