# 装配专项视觉复核与 synthetic 调试建议

2026-09-22，审阅者：GPT-6 Astra Max，任务 `m2_assembly_astra`。观察 ID：`OBS-M2-20260922-ASTRA-ASSEMBLY-01`。本轮实际查看实录、CAD 六视图、左右手网格和两种 arm link6 网格，生成完整三角面的 CPU 多视图并做确定性几何检查。**当前机械装配未通过；批准范围仅为所列观察与 synthetic 调试候选。**

初始 `combined01` 有两个明确问题：arm link6 原点不在所用几何端面，转接件明显悬空；D405 被放在掌宽侧面，光轴朝手指方向的侧方。修正相机姿态与平移可以改善调试视野，但不能补出裸腕法兰、真实配合面或缺失支架。

## 输入版本与证据范围

公开文档工作分支从 `949ab2e68a10d6f2f49f81b3e75cb64866971764` 派生。本轮只读主工作区输入；其未提交配置按文件 SHA256 冻结在私有观察包中，不把 branch HEAD 当作这些输入的完整版本。

| 输入 | 实际使用范围 |
|---|---|
| 实录批准包 `OBS-M1-20260922-ROOT-01` | 再查看第二段 0.37 s、第一段 3.86 s、12 帧全景接触表；核对帧哈希 |
| CAD 批准包 `OBS-M1-20260922-ASTRA-FLANGE-01` | `run04` 的 BRep 报告、六视图和米制 OBJ；本轮复用几何，不重新解析 STEP |
| OmniHand 2025 左、右包 | `OmniHandleft3.urdf` / `OmniHandright4.urdf` 及其各自 mesh；不使用镜像缩放替代另一只手 |
| DISCOVERSE `d67f47c084aba0e0cf422a8725235f8b9238655a` | `new_airbot_play` 的 MJCF 与匹配 OBJ；`combined01` 使用其固定关节的静态视觉快照 |
| ROS2 候选 `7792960fb60f3118d9827b641dcc441e9ae2d06f` | `airbot_play_description` 的真实 URDF、link6 visual；本轮只做末端网格比较，不将前述静态快照视为该 URDF 的动力学转换 |
| RealSense ROS `9a11121700cb4780e273e34141f6402fe184321d` | `_d405.urdf.xacro`、`d405.stl`；STL 以 0.001 缩放，内部外参只用官方 nominal；不采用上游明确标为不可靠的惯性 |

详细输入映射、原始来源哈希、派生图像、计算脚本和观察包位于 ignored `.local/evidence/m2/astra_assembly/`。公开提交不包含私有 CAD、网格、实录或原始来源路径。输入版本或姿态改变后，应重新检查受影响的结论。

## 实录与 CAD 支持的结论

0.37 s 近景可见手腕连接处的独立金属构件、紧固件和手根下方的银色相机安装总成；3.86 s 从另一视角支持该偏置安装关系。结合用户明确说明，可采用“两手下均有 D405”的拓扑。镜头正面、全部机械接触面和紧固路径没有被完整展示，不能从这些画面恢复相机旋转、搭接深度、法兰 datum 或米制平移，也不能确认实物的解剖手侧与模型修订。

所交 CAD 是中空带槽的套式单件。它未包含延伸至下置相机的支撑结构，也不能被视为近景中全部金属安装件的完整模型。CAD 负 Y / 正 Y 两端可以被定义为审查端面，但实际哪端接臂、哪端接手以及绕轴键位仍是 unknown。详见 [单件 CAD 审查](FLANGE_ASSEMBLY_REVIEW.md)。

## arm、mount、hand 的 datum 检查

CAD 到当前 mount mesh 的变换将主轴 CAD +Y 映射到 mount +Z，将 CAD 负 Y 端放在 mount Z=0，正 Y 端放在 Z=26.1 mm。该坐标重表达正确保留了原始几何；它本身不确认两端机械归属。

左右 palm 的原生 URDF 坐标都沿 +Z 指向四指，Y 主要为掌宽方向，X 为掌面法向。两侧 palm STL 的底部 Z 分别约为 -0.04287 / -0.04209 mm，最高约 113.45 mm；根部存在绕该轴的圆形孔口。因而让 hand root +Z 与套筒轴同向是合理的 **synthetic 同轴候选**。零绕轴角、26.1 mm 端面对接和无插入深度仍只是设计假设，孔组、螺钉、防转键及真实接触没有验收。

arm 侧问题更大。`combined01` 将 `link6` 原点直接当作 flange；但其 visual AABB 为：

| 网格 | X 范围 / mm | Y 范围 / mm | Z 范围 / mm | 三角面 |
|---|---:|---:|---:|---:|
| DISCOVERSE new link6 | -28.5～28.5 | -69～69 | -195.95～-51.4 | 14,684 |
| ROS2 link6 | -134.22～28.5 | -69～69 | -195.95～-51.4 | 16,802 |

两者都保留原平行夹爪的宽横梁/座。将三角顶点按 1 µm 取整后，前者全部 14,684 个三角面都包含在后者中；后者额外 2,118 面形成负 X 侧的支架样构件。删除两个 gripper finger 子 link 并不会删除这些内嵌结构，也不会自动得到裸腕质量/惯性。

Z=-51.4 mm 仅是最前包围面。中心轴的前向 mesh 交点为 Z=-79.5 mm；在套筒环壁对应的半径上，前向交点也不共面。因此 **不能把 bbox 最大 Z 当作已识别的 flange 接触平面**。初始 mount 从 Z=0 开始，距所有 link6 几何至少 51.4 mm；向负 Z 移动只能修正显示位置，仍不能证明配合。

## D405 坐标与候选参数

本轮 `camera_housing` 指官方 D405 `bottom_screw_frame`，不是外壳中心。官方 nominal 链合成得到：

```text
T_housing_color_optical.translation_m = [0.01085, 0.009, 0.021]
T_housing_color_optical.rpy_rad       = [-π/2, 0, -π/2]
T_housing_color_optical.wxyz          = [0.5, -0.5, 0.5, -0.5]
```

这使 optical +Z 朝 housing +X。初始 `T_mount_camera_housing` 为 `[0,-0.055,0.012]`、零旋转，所以镜头朝 mount +X，处于掌宽的 -Y 侧；它不能实现“位于掌面下方、朝手指前方”的调试目标。

旧 `combined01` 的显示修正记录于私有 `synthetic_recommendations_v2.json`。下表保留用于重现实验；**其中 arm 侧 -51.4 mm datum 已由下文用户单臂包的新候选取代，不得迁移到新资产。** 以下均采用列向量、米和弧度；生产参数继续独立保持 null。

| 变换 | 候选平移 / m | 候选旋转 | 来源与限制 |
|---|---|---|---|
| link6 → visual envelope datum | `[0,0,-0.0514]` | identity | 来自 mesh 最前包围面；**不是物理 flange**，中心仍悬空 |
| visual envelope datum → mount | `[0,0,0]` | identity | 显示设计；沿用 CAD 负 Y 端朝臂、正 Y 端朝手的未确认假设 |
| mount → hand root，两侧分别记录 | `[0,0,0.0262]` | identity | 26.1 mm CAD 长度加 0.1 mm 显示间隙；不是装配公差或实测长度 |
| mount → 左 housing | `[0.080,0.011,0.012]` | RPY `[0,-π/2,0]` | 设计值，光心落在 mount `[0.059,0.020,0.02285]` |
| mount → 右 housing | `[0.080,-0.029,0.012]` | RPY `[0,-π/2,0]` | 设计值，光心落在 mount `[0.059,-0.020,0.02285]` |
| housing → optical | 上述官方 nominal | 上述官方 nominal | 保留来源，不称已标定 |

两侧 housing 的旋转 wxyz 均为 `[√0.5,0,-√0.5,0]`。合成后 optical +Z=mount +Z、optical +X=-mount Y、optical +Y=mount X。外壳位于 mount X=38～80 mm，在所审阅 q=0 机械臂快照中，mount +X 约为世界向下。相机左右偏置分别使光心避开对应拇指；这是可追溯的设计选择，不是复制或镜像一侧标定。

相机到套筒之间没有被建模的承力件。这组变换只规定空间关系，不能称为已完成机械安装。真实支架、孔位、螺钉、载荷及惯性必须另行解决。

## 干涉与遮挡结果的适用边界

CPU 检查使用完整 visual 三角面，未抽面；多视图采用正交投影和简单面排序，不是相机 RGB 或物理渲染。对于 DISCOVERSE 静态快照及所列 hand pose，v2 相机外壳 AABB 与所有其他部件的 AABB 均分离，故这些 visual mesh 在该姿态下不相交。此结论不包含尚缺的相机支架/线缆、PhysX 碰撞代理、其他关节姿态或 ROS2 整条 arm 的 FK。hand 与套筒的显示间隙也不构成紧固或接触验收。

为比较设计视野，使用 640×480、fx=fy=600 px、主点 `[320,240]` 的理想针孔值；在 u=8～632、v=8～472 上采样 21×15 条射线，检查 optical Z=0.01～0.5 m 的首个自身网格交点。它们不是实测 D405 内参，统计是采样射线比例，不是精确像素遮挡率。

| 相机设计 | 左手阻挡射线 | 右手阻挡射线 | 中心射线 |
|---|---:|---:|---|
| 朝前、两侧 housing Y 均为 0 | 64/315 | 162/315 | 两侧畅通 |
| v2，分别避开拇指 | 17/315 | 20/315 | 两侧畅通 |

v2 明显改善这个静态姿态下的自身遮挡，仍未检查任务盒子、中转区、框子或全运动范围的覆盖。初始 arm q=0 + base yaw=0 时，手指与修正后的光轴约朝 world +X，目标区主要在 +Y；不能据上述统计宣称桌面覆盖通过。

本轮 hand 状态沿用 preview 的独立 q=0 加 URDF mimic。左 `l_thumb_roll_joint=0` 低于其 URDF 下限 `0.03490658 rad`；右侧此项范围检查没有超限。遮挡数字只描述这份静态网格，不构成有效物理关节态的验收。后续应选定限位内姿态，再重做遮挡与碰撞检查。

## 续审：官方裸腕图与用户单臂包

用户指出旧夹爪座不应保留，并提供[官方机械结构说明](https://docs.discover-robotics.com/airbot-play/quick-start/mechanical-structure.html)及一份单臂 `play` URDF 包。本任务实际查看用户截图、官方高清 J6 图、[官方末端执行器安装说明](https://docs.discover-robotics.com/airbot-play/quick-start/installation/end-effector.html)、其中的末端连接件/夹爪连接面照片，以及用户包 link4～link6 的原始三角面。新增观察 ID：`OBS-M3-20260922-ASTRA-BARE-WRIST-02`；来源映射、哈希和图示存 ignored `astra_assembly/bare_wrist02/`。

官方说明明确 AIRBOT Play 有 J1～J6 六个关节；末端连接件出厂默认安装，G2/E2 是可选末端。安装页要求把夹爪的连接固定位放入连接件，再以侧面四枚 M3 螺丝固定；相机连接件位于末端连接件与夹爪之间，其图纸需向技术支持获取。照片显示带水滴状限位的框形连接件。**这些信息确认了裸腕/连接件与可选夹爪的边界，但没有给出私有 OmniHand 转接件的装配尺寸。**

官方组织页面能追溯到 AIRBOT Play Hardware / MoveIt2 仓库；在用户提供新包后停止扩展公开模型搜索。本轮没有把新包宣称为从官方仓库下载或已绑定实物修订。它是用户提供、经过当前几何复核的候选，来源身份与生产验收分别记录。

### 新包保留完整第六轴

用户包的 `joint6` 是 `link5 → link6` 的 revolute joint，原点平移为零、RPY 为 `[-1.5708,0,0]`、转轴为 `[0,0,1]`；其 datum 与旧模型 `joint6.xyz=[0,0.23645,0]` 不同。应整体使用新 URDF 的运动链和匹配 mesh，不能只换一件 link6 网格却沿用旧关节原点。

本轮只读用户包的 URDF 和 link4～link6 三件 mesh，没有裁切或重写源资产。CPU 多视图显示 link5 包含末端较大的电机/腕部外壳，link6 是主要处于该外壳内部、末端略伸出的近圆柱形输出件。**visual 分件所属 link 不能当作执行轴数量：必须保留源 `joint6`、`link6` 和相应转轴，不能把手直接固定到 link5 来获得“五轴裸腕”。** 官方照片中的完整框形连接件也未被这份 link6 网格证明已包含。

在源 link6 坐标中，786 个三角面的包围范围为 X=-17.81155～17.67223 mm、Y=-17.69878～17.76184 mm、Z=46.98305～86.51586 mm；没有旧候选的 138 mm 宽夹爪横梁。该变化来自替换整套来源，不是任意切除旧网格。

`end_joint` 的平移为 `[0,0,0.0865]`，但 RPY 为 `[1.57,-1.57,0]`，因此 end frame +Z 近似朝 link6 -Y，不能作为与原生 link6 同轴的 identity flange frame。建议直接从 link6 接调试 mount；若改用 end_link 作父节点，必须显式乘完整逆变换，私有 JSON 已保存该等价矩阵。

### 新显示候选及插入限制

`nas_synthetic_chain.json` 替代旧 arm 侧偏移。针对本轮设计的 base yaw=π/2、q2=-0.7、q3=0.7、其他 arm q=0：

| 变换 | 新候选 | 依据 |
|---|---|---|
| link6 → mount | 平移 `[0,0,0.0866]`；RPY `[0,0,π/2]`；wxyz `[√0.5,0,0,√0.5]` | 86.5 mm 源 end_joint 名义轴向位置，加 0.1 mm 显示净空；90° 绕轴角是 synthetic 设计 |
| mount → hand root | 平移 `[0,0,0.0262]`；identity | 沿用套筒长度及显示间隙假设，两端归属/键位仍未知 |
| mount → housing | 暂沿用上文 v2 的两侧相对链 | 新整臂姿态与手关节状态必须重新检查遮挡；不复用旧姿态的遮挡通过结论 |

确定性 FK 得到：上述 arm 姿态的 link6 +Z 约朝 world +Y，link6 +Y 约朝 world -Z。若直接使用零安装旋转，手掌和旧 v2 相机会朝侧面。加入 +90° 后，mount +X 约朝世界下方、手指/光轴 +Z 朝桌面 +Y。这个绕轴角仅实现批准的下置拓扑，不证明实物螺孔或防转键就是该方向。

候选保留原 CAD mesh 的坐标重表达；其下端与 link6 网格最高点有约 0.08414 mm 的轴向显示间隙，与 link5 最高点约有 1.07386 mm 间隙；套筒上端与左 palm mesh 下端约有 0.05713 mm 间隙。这些是三角面坐标计算结果，不是实物间隙、制造公差或完整碰撞验收。

新 link6 主体横向直径约 35.5 mm，而私有 CAD 的主要内圆柱直径为 28.5 mm。该孔径关系不支持直接把套筒向后推入并包覆 link6 主体；不能为了画面贴合随意扩大孔径、裁掉输出件或关闭碰撞。本轮采用无插入的显示候选，仍须确认实际配合件、止口、两端归属、搭接深度、孔位及紧固方法。端面对接也不构成机械承力方案的批准。

几何图为 `nas_wrist_views.png`、`nas_link6_sections.png` 和 `nas_candidate_chain.png`。它们显示源分件与候选链，未附加物理驱动。新包六轴 `effort` / `velocity` 为零，动力学输入仍由负责人另行解决；不能因裸腕外观改善而宣称运行时机械验收通过。

随后实际审阅主会话 Blender 5.2.2 LTS / CPU Cycles 的 `combined03 → preview02 → blender_review03` 输出，以及最终 `assembly_before_after.png`、`wrist_details.png`。最终图中旧宽夹爪座已消失，源腕部、第六轴、橙色套筒和蓝色下置 D405 的显示关系符合上述设计。相机承力支架仍缺；画面不能证明紧固、间隙或碰撞通过。颜色是主会话的审阅替代材质，渲染为静态装配复核，不是训练 RGB 或生产外观标定。

## 已执行、未执行及交接

以下命令在主工作区根目录使用已有独立 CPU 环境执行成功；没有新增依赖、启动 Kit/GPU 或修改其他代理的输出。命令和版本另存私有 `commands.json` / `validation.json`。

```powershell
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/review_geometry.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/analyze_datums.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/propose_synthetic.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/compare_arm_sources.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/camera_alternate.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/finalize_review.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/bare_wrist02/review_nas_wrist.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/bare_wrist02/candidate_chain.py
& .local/evidence/m1/flange/venv/Scripts/python.exe .local/evidence/m2/astra_assembly/bare_wrist02/finalize_bare_wrist.py
```

本任务未运行 Blender 导入、Isaac/PhysX cooking、动态碰撞、接触抓取、标定板投影、传感器 RGB 或 LeRobot 写入/回读。主会话其他任务的运行结果需各自记录，不归入本轮 CPU 审查的通过项。

下一接口是：负责人应用新包 synthetic 候选并绑定实物版本及生产 datum；补齐转接件两端配合、键位、插入深度及相机支架；在有效关节态下验证所有碰撞/视野；完成独立的质量、惯性和左右实测外参。`T_flange_mount`、`T_mount_hand_root`、`T_mount_camera_housing`、`T_housing_color_optical` 的生产值继续为 null，采集关闭。公共 schema、成功判据和生产门禁未在本任务中修改。
