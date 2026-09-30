# 裸腕模型与 Thor 桌面修正

用户指出旧预览保留了不应存在的夹爪座，并提供私有 `play` 单臂包。此轮从模型绑定修正，不将旧网格任意切割后称为官方裸腕。

## 单臂输入

`urdf/play.urdf` SHA256 为 `b02c7ac6fd1dd6a65f355ef6650c2a7584fdbd56ccfc497b767292010ce4e0b0`。包有 8 个 link、6 个 revolute 和 1 个 fixed；7 个 STL 的 visual/collision 引用闭合。`link6` 后只有 `end_joint → end_link`，原 G2 夹爪子树不存在。来源按用户提供的私有 SolidWorks 包记录，不自动称为已验证官方发布版本。package.xml 声明 Apache-2.0，但不将其等同于独立核实的来源与再分发授权；所有网格留在私有存储。

旧候选的 `link6` 网格包含原夹爪座，删下游手指不能消除此残留。新包必须使用自己的坐标链，不能沿用旧 `-51.4 mm` 包围面偏置。官方机械结构与末端安装说明是几何复核依据，不提供当前实物标定。

六个关节的 effort、velocity 都为零，关节惯性、方向和驱动参数未通过物理验证。静态模型替换不解除此阻塞，生产参数仍为 null。

## 接口与验证范围

`resolve_preview_positions` 为每个独立关节选择限位内、最接近零的显示默认值，允许显式具名姿态；拒绝超限、非有限数、未知名称和覆盖 mimic 关节，并检查 mimic 结果。这样不再用超限的左拇指零位评估装配。它不定义机器人 home 或 action。

`build_commissioning_preview.py --table-usd <private-path>` 引用选定桌面 USD，只读访问源文件。profile 必须明确资产 ID、源单位/up axis 与 `T_world_asset`；工具核对源 metadata，把坐标转换到米，不改变桌子的实际尺寸，也不在缺绑定时静默回退成方块桌。

Blender 输出包含三路静态相机及可选 `--wrist-closeups` 左右腕部审阅视图。近景是额外审阅相机，不属于训练三路 RGB。图像、派生 USD/BLEND 和机器路径只留在 ignored 证据目录。

本轮不以静态预览宣称机械紧固、碰撞、相机覆盖或接力成功。具体端面与安装候选见 `ASSEMBLY_VISUAL_REVIEW.md` 的补充复核。

## 当前 synthetic 候选

`T_link6_mount` 为平移 `[0,0,0.0866]` m、RPY `[0,0,π/2]`；采用源 end_joint 的0.0865 m名义端面并加0.1 mm显示净空。旋转是为当前显示姿态安排掌下相机的设计clocking，不是实测键位。mount→hand_root 保持0.0262 m端面对接候选。link6直径约35.5 mm，大于CAD主要内径28.5 mm，不能把套筒直接重叠套入；机械配合未通过。

两底座位于同侧，synthetic base yaw为π/2，J2=-0.7、J3=0.7 rad，其余独立关节取限位内默认值。原生Thor桌保持物理尺寸，平移x=-0.2737505042552948 m居中；双底座x=±0.23、y=-0.22 m，桌外框子中心x=0.64 m。这些是调试布局，不替代实测标定。
