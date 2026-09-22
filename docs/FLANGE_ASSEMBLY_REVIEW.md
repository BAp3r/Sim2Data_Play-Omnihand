# 转接件 STEP 几何审查与仿真安装

2026-09-22，审阅者：本任务 GPT-6 Astra High。输入为用户提供的一件私有 STEP，源文件 SHA256、来源映射和派生几何只存 ignored 证据目录。观察 ID：`OBS-M1-20260922-ASTRA-FLANGE-01`，批准范围为 **CAD 几何与候选装配拓扑**，不批准实物适配或生产外参。视频拓扑沿用 `OBS-M1-20260922-ROOT-01`：双臂同侧、独立转接、两手下各有 D405；这些说明不证明本 CAD 已覆盖整套安装总成。

## 输入和实际检查

源文件 482,716 字节，STEP header 标注 CREO PARAMETRIC、2025-12-10。文件名不是实物手型/修订的证明。OpenCascade 读取到长度单位 millimetre、角单位 DEGREE、立体角 steradian；STEP reader 显式以毫米为内部长度单位，转换后解析曲面参数角为弧度。项目最终使用米、弧度、wxyz 和 `T_A_B` 列向量约定。

本机及现有服务器环境没有可调用的 Blender/FreeCAD/OCP/OCC。经负责人同意，创建独立 CPU 环境，没有改已有仿真环境或安装系统包。实际版本：Python 3.12.11、`cadquery-ocp-novtk==8.0.1.0.0`、`numpy==2.5.3`、`matplotlib==3.11.2`；完整 freeze 和命令保存在私有证据目录。

`scripts/inspect_flange.py` 以 `BRepBndLib.AddOptimal_s(..., useTriangulation=False, useShapeTolerance=False)` 求已裁剪实体的几何包围盒，而非取 STEP `CARTESIAN_POINT` 极值；结果仍受 CAD/数值容差约束。脚本同时输出所有面类型、平面、圆柱轴、面积和面包围盒，使用 Agg CPU 绘制六个正交视图。面编号是本次固定源文件及导入器的遍历编号，不是跨修订永久 ID。

## 可确定的单件几何

| 项目 | 结果（CAD frame；单位 mm，另有注明除外） |
|---|---|
| 实体 | 1 个 solid，134 个 face；BRepCheck 有效 |
| AABB 最小点 | (93.061107475, -38.218873457, -136.191237799) |
| AABB 最大点 | (130.629179225, -12.118873257, -98.623166049) |
| AABB 尺寸 X/Y/Z | 37.568071750 / 26.100000200 / 37.568071750 |
| 主要轴线 | 平行 CAD Y，穿过 X=111.845143350、Z=-117.407201924 |
| 外圆柱面 | 半径 18.784035775，面 20、46；存在削平/切槽，不是完整圆筒 |
| 主要内圆柱面 | 半径 14.25，面 12、43；不是全长无缺口的简单孔 |
| 两者同轴圆柱区径向厚度 | 4.534035775；局部削平、开孔、切槽处不适用 |
| 实体体积 | 10,270.571057 mm³，仅几何体积，没有材料密度假设 |

六视图显示一件中空、局部开口的套式转接件：两端轮廓不同，外壁有削平和径向孔，一端有局部凹槽/缺口。它可以作为候选转接件 mesh；不能用实心圆柱或 40 mm 垫块替代其碰撞外形。**26.1 mm 是本件 CAD 轴向总范围，不是已确认的 arm flange → hand root 距离**，也不证明用户“总装约 4 cm”估计错误。

下表给出可用于对图的候选几何 datum。为方便读数，临时定义 `cad_review_origin` 为主要轴线与 CAD Y=-38.218873357 平面的交点，轴方向仍与原 CAD 相同。这只是图纸审查坐标，不赋给生产 `mount_assembly`。

| 特征 | 几何证据 | 装配解释边界 |
|---|---|---|
| 负 Y 端平面 | 面 30，Y=-38.218873357，面积约 308.379 mm² | 候选端面；不能自行命名为机械臂侧 |
| 正 Y 端局部共面区 | 面 70、78、127，Y=-12.118873357，与前项相距 26.1 | 候选端面；不是完整连续平板 |
| 正 Y 端凹槽底面 | 面 73–75，Y=-14.618873357 | 相对正 Y 端下凹 2.5；不能推断紧固件规格 |
| 小径向孔组 | 面 13–16、61–64，直径 2.5；轴向站位为 review Y=3 | 四个壁面开口，沿 CAD X/Z 两条穿心轴线；CAD 未证明螺纹 |
| 较大径向孔组 | 面 23、24、27、28、44、48、51、52，直径 3.2；站位为 review Y=20.1 | 四个壁面开口，沿 XZ 平面中两条方向为 (√3/2,0,±1/2) 的穿心轴线 |

圆柱面会被 STEP 拆成多片，面数不是孔数。孔轴位置及直径可确定，但螺纹、公差、螺钉长度/头型、装配预紧、材料、表面处理和实物磨损均不能由这些无注释几何推断。中央中空不代表该处必然穿线；相机支架也未被识别为本 solid 的一部分。

## Blender 中保真安装

1. 只读保留 STEP；用本脚本输出私有 `flange_cad_m.obj`。脚本采用 0.05 mm 线偏差、0.15 rad 角偏差离散化，顶点数值已除以 1000，保留 CAD 原点和轴。OBJ 本身无可靠单位元数据，单位依赖报告，不要再乘 0.001。
2. Blender Scene Units 设 Metric / Unit Scale 1；导入 OBJ global scale=1，显式关闭额外轴置换或设源/目标同为项目 Z-up。不同 Blender 版本的 importer 轴选项不同，应以导入后的坐标/bbox 核对，不能只看“朝向正确”。导入应约为 0.037568 × 0.026100 × 0.037568 m，离散化容差内一致。导入对象不得自动移到质心、任意旋转 90° 或负比例镜像。
3. 在审查副本中创建 `cad_frame` Empty，原始 mesh 保持相对坐标不变；需要移原点时另建 `cad_review_origin` 并记录完整 `T_CAD_review`，采用保持世界变换的重挂接。review 原点不是 flange/hand datum。不要把 CAD 中离原点约 0.1 m 的偏移当成机械装配偏移。
4. 导入审核过的 AIRBOT Play 与 OmniHand 资产，确认末端物理 link、flange frame、hand root 和修订/解剖手侧。对接时先匹配真实接触平面与轴线，再用孔组/防转结构消除绕轴自由度；必须由装配图、配合件 CAD 或实测确认两端归属、插入/搭接深度和角向键位。单凭这件 CAD 的总长不能完成对接。
5. 分别建立 `arm_flange → mount_assembly → hand_root` 和 `mount_assembly → camera_housing → color_optical` 的 Empty/frame 链。D405 下置拓扑已确认，但 camera housing 几何和相对于本件的安装孔/支架尚缺，不用目测把相机附到 palm 原点。左右两套分别记录，不能复制一侧标定或镜像手模型。
6. 把已确定的变换以米、wxyz 序列化；仍未知的平移或旋转整体保持 null。冻结变换后再导出 USD 并检查 `metersPerUnit=1`、Z-up、object scale、轴置换与 hierarchy；保留审查 frame。Blender 中看起来贴合只是辅助审查，不能替代配合检验和标定。

组合始终为：

```text
T_world_hand_root = T_world_flange × T_flange_mount × T_mount_hand_root
T_world_color = T_world_flange × T_flange_mount
                × T_mount_camera_housing × T_housing_color_optical
```

USD/Blender Camera 本地 -Z 前向、+Y 上向；常见 color optical 是 +Z 前向、+X 右、+Y 下。若采用这两个确切约定，同原点转换为绕 X 旋转 π（wxyz=[0,1,0,0]）；这仅是 camera 表达约定转换，**不提供** D405 housing → optical 标定。最终仍需按 Isaac Lab 所选 convention 显式适配并做标定板投影检查。

## Isaac USD 物理处理

visual 使用原始中空细节，collision 使用经净空检查的多个凸件/凸分解。单一 convex hull 会封掉中空和缺口，可能制造错误碰撞；也不能为避免穿透而关闭必要碰撞。动态 articulation 安装件的三角网格碰撞支持及 cooking 行为依选定 PhysX/Isaac 版本验证，不默认直接拿高精 mesh 当动态碰撞。

在官方 flange 物理 link 上建立固定装配关系，保持 frame、碰撞、质量归属一致。可保留独立固定刚体，或在确认刚性连接后将其质量/惯性合并入既有刚体；不能给同一固定载荷重复设置两份质量，不能无审核嵌套 rigid body API，也不要改变原 arm/hand 的 articulation 根和驱动。相机外壳的碰撞/质量与光学 Camera prim 分离。

本 CAD 没有可信材料密度和实物载荷清单，所以质量、质心、惯性仍未验收。几何均匀体积质心可作审计辅助，不能冒充实物质心。确定材料和紧固件后，逐件计算/测量，在统一 frame 中旋转惯性张量并用平行轴定理合并，记录惯性单位 kg·m²；相机、支架和紧固件各计一次。正式运行前检查净空、初始穿透、关节范围内自碰撞、载荷及相机遮挡。

## 证据、验证与阻塞

私有证据位于 `.local/evidence/m1/flange/`，`observation.json` 关联源 hash、`run04/geometry.json`、`run04/cad_views.png` 的六视图位置和审阅状态。`run04/flange_cad_m.obj` 是未经公开再分发授权的派生 mesh；不随仓库提交。`commands.json` 保存实际执行命令、早期 OCP 8 接口适配失败和最终成功轮次；`requirements.freeze.txt` 锁定实际环境包。

已执行：真实 STEP read/transfer、BRepCheck、解析面/bbox/volume、CPU 多视图、OBJ 单位和有向体积核对。OBJ 共 9,256 顶点/10,232 三角面，mesh 与 BRep 最大 bbox 差约 0.004236 mm，有向体积相对差约 0.023585%，索引和阈值检查通过。脚本 `--help`、语法检查通过。可复用调用（私有路径通过参数传入）：

```powershell
python scripts/inspect_flange.py <private-step-path> <new-private-output-directory>
```

未运行 Blender 导入、Isaac USD cooking/场景打开、GPU 渲染、碰撞/载荷或抓取测试；训练 GPU 占用期间未启动 Kit。以上是 CPU CAD 几何审查，不是物理抓取、装配标定或 M2 验收。

仍需：配合件和实物版本、法兰/手根 datum 对应、两端装配方向与搭接深度、紧固件、相机安装件与光学外参、材料和质量/惯性。`T_flange_mount`、`T_mount_hand_root`、`T_mount_camera_housing`、`T_housing_color_optical`、左右解剖手侧均保持 null；生产采集继续关闭。
