# B_SCENE 装配输入模板

本文件定义装配实现轮次需要交给场景构建器的输入边界。它只消费批准的拓扑观察和用户明确说明；不从画面估计尺寸、姿态或标定。对应的可复制草案是 `configs/assembly_inputs.template.json`。这是需求交接模板，不能直接合并进 `scene_spec` 或当作已标定场景；生产开关为 false。

批准观察包：`OBS-M1-20260922-ROOT-01`，状态 `approved_topology_only`，审阅者为主会话。它确认双臂位于桌面同一侧、法兰与转接件分离、审阅近侧画面可见手根下置腕相机总成、桌面上方有独立主相机支架；用户另确认两只手下均安装 D405。它不能解析米制尺寸、SE(3)、手/臂修订、解剖手侧、内参、质量/惯性或驱动映射。

## 每侧原子链

左右侧分别填写，不复制一侧的值到另一侧：

```text
world → arm_base → ... → arm_flange
                         └→ mount_assembly
                             ├→ hand_root → grasp_tcp
                             └→ camera_housing → color_optical
```

场景工具按以下顺序组合列向量变换：

```text
T_world_hand_root  = T_world_flange × T_flange_mount × T_mount_hand_root
T_world_color      = T_world_flange × T_flange_mount
                      × T_mount_camera_housing × T_housing_color_optical
```

`T_flange_mount`、`T_mount_hand_root`、`T_mount_camera_housing` 和 `T_housing_color_optical` 是独立的生产输入。每个输入必须包含 `translation_m` 和 `rotation_wxyz`；缺任一项就保持 `null` 并阻断场景装配。`T_flange_hand_root` 与 `T_parent_color_optical` 只能作为组合后的核对值，不能反向补齐原子链。

每侧 `installation_bodies` 分别列出 mount_assembly 与 camera_housing 的质量、质心、惯性参考 frame 和碰撞几何。不能重复计入相机质量；只有明确焊接刚体关系后，才能用平行轴定理合并载荷。模板的全局 adapter 名义信息不替代两侧安装输入。

约 4 cm 是转接长度的名义描述。它不提供平移轴方向或旋转，不能写成 `[0, 0, 0.04]`，也不能替代转接件/相机安装件的质量、惯性和碰撞几何。

## 待补输入

- 桌面尺寸和相对 floor 的表面高度（`world` 的 z=0 仍是桌面表面），左右底座在 `world` 中的位姿，以及两臂对应的 AIRBOT Play 资产/修订。
- 两侧 OmniHand 2025 变体、硬件修订、解剖手侧、TCP 定义和可观测驱动映射。
- 两侧转接件、相机 housing 和支架的尺寸、质量、惯性、碰撞几何及上述完整 SE(3) 链。
- 两只 D405 的 RGB 流配置、分辨率、内参、畸变、裁剪/对齐规则、时间标签和 color optical 外参；depth 目前关闭。
- 通用 overhead 相机的世界位姿与覆盖验证。分辨率、针孔内参和理想畸变可以作为 `synthetic` 设计值，但不能代替位姿或覆盖检查。
- 桌面中转区、桌外右侧框子的世界位姿/内尺寸/壁厚/支撑高度、右臂可达性，以及源物体尺寸、质量和刚性假设。

输入齐备后才运行 Isaac 场景打开、初始穿透、遮挡、标定板投影和可达性检查。本模板和 CPU 链测试不构成 M2 或物理抓取验收，生产采集继续关闭。

## 已批准单件 CAD 观察

`OBS-M1-20260922-ASTRA-FLANGE-01` 已由 Astra High 实际解析和审阅，详见 `FLANGE_ASSEMBLY_REVIEW.md`。CAD 约为 37.568 × 26.100 × 37.568 mm 的中空带槽转接件。26.1 mm 不作为 flange→hand 平移；两端归属、搭接、键位、左右安装和质量仍待实测/配合件确认。派生 mesh 数值已为米，保留原 CAD frame，禁止二次缩放或把 CAD 原点偏移当装配外参。
