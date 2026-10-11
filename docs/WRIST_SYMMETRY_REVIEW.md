# D405 左右对称与 OmniHand 右手装配复核

**当前配置已在 M5 按用户要求把两台 D405 各向外移动 20 mm**：左 housing 的 mount Y 从 +11 mm 改为 +31 mm，右从 -11 mm 改为 -31 mm；其余安装变换及右机身 180° 滚转保留。当前外观与外移复核见 [M5 实录参考与相机外移](PHOTO_APPEARANCE_REVIEW.md)。下文保留 M4 的原始候选、数字和验证范围，用于追溯，不代表当前相机偏置。

2026-09-22，实际审阅者：GPT-6 Astra xhigh，子任务 `m4_symmetry_astra`。输入基线 `63e4a213563c60e822dc08bbbaff869359d9032c`；观察 ID：`OBS-M4-20260922-ASTRA-WRIST-SYMMETRY-01`。本轮批准范围为 **synthetic 静态装配候选及下面的观察事实**，机械装配和生产标定仍未通过。

右手已经使用独立的 `OmniHandright4.urdf` 和右侧 STL。需要修正的是 D405：上一版只对称了彩色光心，两台相机保持相同绕轴角，导致外壳中心不对称。新候选将右侧真实 D405 绕光轴转 180°，同时补偿底部螺孔原点到外壳中心的偏移，使外壳包络中心和光心同时对称。两侧仍使用同一份真实 D405 STL、正缩放及各自右手坐标系。

## 输入、来源与真实右手核验

本轮实际查看前版 `wrist_details.png`、`overhead.png`、`review_left_wrist.png` 和 `review_right_wrist.png`，随后查看新生成的前后比较及左右四视图。前版近景能看到蓝色相机在两只手下，但相对掌根及拇指的横向位置不同；照片视角本身不能确定米制偏移或完整外参。

模型来源沿用 [模型绑定记录](MODEL_BINDING.md) 和 [前次装配复核](ASSEMBLY_VISUAL_REVIEW.md)。OmniHand 来自[官方 O10 页面](https://www.agibot.com.cn/DOCS/OS/Omnihand-O10)的独立左右 URDF 包。本轮没有重复下载；重新读取本地已绑定来源的 URDF、对应网格与组合 URDF，检查实际消费链。

| 场景侧 | 原生 URDF / 根 link | 本轮读取的 URDF SHA256 | 组合模型核验 |
| --- | --- | --- | --- |
| 左 | `OmniHandleft3.urdf` / `l_palm` | `8c33ee9e9e9ce08cb9e379618d7e06f07791bf65a159e36f6481dd4e4d75e468` | 18 links、17 joints；18 个 visual 均解析至左包 |
| 右 | `OmniHandright4.urdf` / `R_palm` | `5acb477166ae7b1211d2a8378fd1593e59e7742aa26f4f4578c0ba01bda9e952` | 18 links、17 joints；18 个 visual 均解析至右包 |

两侧每条手关节的类型、父子 link、完整 origin、axis 和 limit 与各自源 URDF 一致；所有网格缩放均为正。上述 joint 数是模型结构统计，不是独立电机或 action 维度。左右手来自不同原生模型修订，不要求其网格逐顶点互为镜像，也不以旋转左手代替右手。

前版 `combined03 → preview02` 的实际静态关节状态作为本轮输入；arm 为 q2=-0.7、q3=0.7，其余 arm q=0，左 `l_thumb_roll_joint=0.03490658 rad`，其他 hand q 沿用 preview。本轮核验这些值均在各自 URDF 限位内。观察图中左拇指位于 mount -Y 侧，右拇指位于 +Y 侧；相机光心分别偏至相反侧。该描述不确认实物的 O10/O12 铭牌、生产零位或硬件修订。

D405 的 `_d405.urdf.xacro` 和 `d405.stl` 沿用 [RealSense ROS 固定 commit](https://github.com/IntelRealSense/realsense-ros/tree/9a11121700cb4780e273e34141f6402fe184321d)，STL SHA256 为 `a248f41149d12b28311829feecbe7a80cf1481fd05e0f5df2c4c7ecd556edd48`。仍使用官方 nominal 内部链；上游明确不可靠的惯性不在本轮使用。

## “对称”具体指什么

采用列向量和 `T_A_B` 映射 B 坐标至 A 的约定。两侧 mount 的 +X 在所审 arm 姿态下约朝世界下方，+Z 沿手指及光轴约朝世界 +Y；以两只对应 mount 的 Y=0 为局部对称面，位置反射矩阵为 `S=diag(1,-1,1)`。结合当前对称的 base 布局，它近似对应桌面场景的世界 X=0 中面。

`camera_housing` 是官方 **bottom screw frame**，不是外壳中心。其名义包络中心为 `[0.00315,0,0.021] m`，彩色光心为 `[0.01085,0.009,0.021] m`；两者在相机横向相差 9 mm。相机壳高 42 mm。真实 STL 的 housing AABB 为 X=-8.35～14.65 mm、Y=-21.09000015～21 mm、Z=0～42 mm，因此包络还有 0.09 mm 的侧向非理想细节，不能把名义箱体当作逐面几何。

本轮要求下置关系、外壳包络中心、两个 nominal IR 光心、彩色光心和光轴方向对称。**不要求螺孔、USB、文字等每个相机特征都是镜像件。** 同一种真实相机不可能用 `det=-1` 的反射作为刚体旋转；镜像整个模型或用负 scale 会生成虚构设备及错误的光学手性。

上版的外壳名义中心为左 `[.059,.011,.01515]`、右 `[.059,-.029,.01515] m`，相差一个额外 18 mm 的横向偏置；彩色光心却是 `[.059,±.020,.02285] m`。这解释了用户看到的外壳不对称。

## 可执行的 synthetic 候选

左侧沿用；右侧围绕向前的光轴滚转 180°。螺孔原点随实体转至外壳另一侧，故同时把右 housing 的 mount X 从 80 mm 改为 38 mm，补偿完整 42 mm 壳高。RPY 按 `Rz(yaw) Ry(pitch) Rx(roll)` 计算；在 pitch=-π/2 时存在欧拉角奇异，多种 RPY 写法可以表达同一个矩阵，不能仅比较某一个角。

| 变换 | 左 | 右 |
| --- | --- | --- |
| mount → housing，xyz / m | `[.080,.011,.012]` | `[.038,-.011,.012]` |
| mount → housing，RPY / rad | `[0,-π/2,0]` | `[π,-π/2,0]` |
| mount → housing，quaternion wxyz | `[√.5,0,-√.5,0]` | `[0,√.5,0,√.5]` |
| housing → color optical，xyz / m | `[.01085,.009,.021]` | 同一官方 nominal 链 |
| housing → color optical，RPY / rad | `[-π/2,0,-π/2]` | 同一官方 nominal 链 |

可复现的推导为：

```text
S = diag(1,-1,1)
A = diag(1,1,-1)
R_right = S R_left A
p_right = S (p_left + R_left [0,0,.042])
```

`S` 和 `A` 只用于求解，不作用于源网格；最终 `R_right` 的 det=+1。右侧 optical +X=mount +Y、+Y=-mount X、+Z=mount +Z；左侧则为 -mount Y、+mount X、+mount Z。因此右相机的原始光学图像相对左侧滚转 180°。必须在相机姿态链中如实保留；不要只转外壳而沿用原来的 optical X/Y，也不要用像素水平翻转代替实体旋转。

| 对称对象 | 新左坐标 / m | 新右坐标 / m | 最大镜像残差 |
| --- | --- | --- | --- |
| 名义外壳中心 | `[.059,.011,.01515]` | `[.059,-.011,.01515]` | 约 3.47e-18 m |
| 彩色 / IR1 光心 | `[.059,.020,.02285]` | `[.059,-.020,.02285]` | 约 3.47e-18 m |
| IR2 光心，由官方 -18 mm 基线合成 | `[.059,.002,.02285]` | `[.059,-.002,.02285]` | 代数上对称 |
| 真实 STL AABB 中心 | 使用全部原始三角面计算 | 使用同一 STL 正旋转计算 | 约 9.84e-14 m |

实际 AABB 中心的旧镜像残差为 18.09 mm。上表微小残差是浮点计算结果，**不是实物标定精度**。完整相机表面的镜像等价未获证明；螺孔原点尤其不镜像：左 screw 在 mount X=80 mm，右在 38 mm。支架必须按两侧实体朝向设计，不能复制旧支架或宣称镜像支架已完成。

## 静态几何和视线结果

本轮对两台相机与各自 arm、mount、hand 的所有 visual 三角面计算 AABB。新候选在所审关节态下与全部这些包络分离，因此这些已提供的 visual 网格不相交。AABB 分离只证明本姿态下所检查网格的分离；尚缺的相机支架、线缆、PhysX 碰撞代理和全行程不在此结论内。

为比较偏置设计，使用 640×480、fx=fy=600 px、主点 `[320,240]` 的 synthetic 针孔，在 u=8～632、v=8～472 采样 21×15 条射线，检查 optical Z=0.01～0.5 m 的首个自遮挡交点。统计采用完整三角面，射线实现的缓存优化与既有实现抽点交叉核验一致。

| 设计 | 左阻挡射线 | 右阻挡射线 | 两侧中心射线 |
| --- | --- | --- | --- |
| 旧版：只对称彩色光心 | 18/315 | 20/315 | 畅通 |
| 新候选：右滚转并补偿螺孔位移 | 18/315 | 20/315 | 畅通 |
| 对照：只将右 housing Y 改为 -11 mm、保持原绕轴角 | 18/315 | 107/315 | 畅通 |

对照方案使名义外壳中心对称，但右彩色光心落在 Y=-2 mm，光心镜像残差为 18 mm，且此姿态下自身遮挡增加。新候选保持原彩色光心位置和前向视线，只改变右相机绕轴角及外壳相对位置。这些计数是采样射线比例，不是精确像素遮挡率，也不是实测 D405 成像、桌面目标覆盖或动态抓取通过。

## 证据、批准观察包与交接

私有证据位于本任务独立 worktree 的 ignored `.local/evidence/m4/symmetry_astra/`。公开文档不嵌入第三方派生网格、CAD、用户实录或原始来源路径。已实际生成并由本 Astra 子任务查看：

| 私有产物 | 内容与图例 |
| --- | --- |
| `wrist_symmetry_before_after.png` | 相同正交视角的左右前后对照；S=螺孔原点，C=名义外壳中心，O=彩色光心 |
| `left_candidate_views.png`、`right_candidate_views.png` | 左右腕部各四视图；灰白=原手 mesh，深灰=原腕 mesh，橙=转接件，蓝=D405；红箭头=光轴/安装轴 |
| `commissioning.symmetry_candidate.json` | 在输入 synthetic profile 上仅实现本轮相机候选及状态说明；不改生产值 |
| `symmetry_transform_patch.json` | 两侧完整相机相对链、旋转推导、限制、null 的生产外参 |
| `symmetry_review.json` | 输入 SHA256、左右源链核验、实际关节态、所有 AABB、315 条射线及镜像残差 |
| `approved_observation.json` | 私有来源/哈希、图像位置、实际审阅者、结论、用户确认、不可推断量、批准范围 |

本轮 CPU 图使用完整三角面的简单面排序与正交投影，只用于装配复核，不是传感器 RGB、训练数据或物理渲染。替代颜色只作图例；官网参考配色及 Blender 新效果由主会话另行生成并记录。

批准观察包确认：已有独立左右手源链被正确消费；旧版外壳中心不对称；新候选显示了左右下置、相反拇指侧偏置和右 screw 朝向变化；给定源几何及姿态的上述计算可供 synthetic 调试使用。用户已确认两只手下均装 D405，并要求右侧对称、使用右手 URDF。实物的完整装配旋转、米制平移、孔组、防转键、支架、线缆、载荷、惯量、传感器内参及外参均不能从这些画面推断，生产值继续 null。

本任务实际执行成功的命令（工作目录为独立 worktree，解释器是既有 CPU 环境，`$workspaceRoot` 从 ignored 部署记录解析）：

```powershell
& "$workspaceRoot/.local/evidence/m1/flange/venv/Scripts/python.exe" .local/evidence/m4/symmetry_astra/review_symmetry.py
& "$workspaceRoot/.local/evidence/m1/flange/venv/Scripts/python.exe" .local/evidence/m4/symmetry_astra/finalize_review.py
git diff --check
```

首轮计算因重复 bbox/BLAS 调用过慢被本任务主动停止；缓存边界与限制本进程线程后完整重跑成功。没有修改公共 cache、全局环境或其他代理输出。本任务未运行 GPU、Kit/PhysX、Blender 导入、标定板投影、接触抓取、相机实测 RGB 或 LeRobot 写入；纯 CPU 检查不替代这些验收。

下一接口：主会话集成候选、保留右光学坐标滚转并重新渲染；另外解决两侧相机承力及螺孔可达性、完整机械配合、质量惯量、实测标定和全行程碰撞/任务视野。公共 schema、真实成功判据与采集门禁由主会话维护，生产采集继续关闭。
