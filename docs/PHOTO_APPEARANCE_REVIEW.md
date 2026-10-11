# M5 实录参考外观与 D405 各外移 20 mm

2026-09-22，实际审阅者：原 GPT-6 Astra xhigh 子任务 `m4_symmetry_astra`。本轮独立分支 `task/m5-photo-appearance-astra` 从 `388382ba7d0f7feff338a80641ff124f22cead83` 派生；观察 ID 为 `OBS-M5-20260922-ASTRA-PHOTO-APPEARANCE-01`。批准范围为 **外观显示与指定单姿态的 synthetic 几何复核**，生产参数继续 null，采集关闭。

## 实际观察和实现

实际查看用户新提供的单张实录截图：前景手处于张开姿态、后景手持物，可见手背和近根部壳体呈银白；机械臂主体深色，长臂有边界连续的浅色侧板。旧预览把 palm 的 local Z≤45 mm 一律涂黑，并用逐三角面法向阈值分黑白，在近景中出现与实录不符的黑掌根、锯齿色界和明显分面高光。

这张截图不提供完整米制尺寸、机械配合、实物内参/外参或精确表面反射参数；没有证据支持修改手或臂的源形状。本轮保留原始 STL/CAD 转换网格，仅改变材质、UV 和显示法线：

- 两只 palm 均使用单一银白壳材质；手指仍浅色，独立金属连接件保持金属近似。
- arm link2/link3 的浅色侧板采用连续角点法向权重与色带纹理，替代逐面二值切分。没有恢复或假称原厂 UV/贴图。
- 45° 限角、角度加权角点法线只影响显示；临时位置归组不写回 mesh，不改变顶点、面索引、机械尺寸或碰撞。
- 生成的色带通过相对路径与 USD 一起携带。实际 Blender 导入确认为 `Non-Color`，最终 `.blend` 已 pack 该图。

M5 首次渲染在纹理复制未完成时启动，出现紫色缺图，故 `blender01` **不计通过**。所有文件就位后独立重导入成功；公共渲染器补充纹理存在/哈希检查、缺图阻断和 pack，再生成 `blender02`。Astra 实际查看了新右腕、全景、俯视，以及主会话制作的左右腕前后对照和同视角双腕图：紫色缺图消失，掌壳为连续银白，旧锯齿黑根和分面亮带得到修正。数字颜色、粗糙度、金属度和臂侧板边界仍是近似。

## 指定相机外移与 FK

用户明确要求两台 D405 各向自身外侧移动 20 mm。保持两臂基座、手、arm q、所有其他装配变换和 `housing → optical` 链不变，仅更新：

| 侧 | mount → housing xyz / m | RPY / rad |
| --- | --- | --- |
| 左 | `[.080,.031,.012]` | `[0,-π/2,0]` |
| 右 | `[.038,-.031,.012]` | `[π,-π/2,0]` |

基于当前 q2=-0.7、q3=0.7 的整条源 URDF FK，左 mount Y +20 mm 对应世界位移 `[-.01999999999964,-1.19222e-7,-5.42479e-9] m`；右 mount Y -20 mm 对应相反位移。世界 X 外移为 20 mm；约 0.119 µm 的横向残差来自源 URDF 近似 RPY 数字，不是测量精度。这里验证的是所列预览姿态，不能将 mount Y 在任意 arm 姿态下都视为固定世界 X。

新名义外壳中心为 mount `[.059,±.031,.01515] m`，彩色光心为 `[.059,±.040,.02285] m`；真实 STL AABB 中心、名义中心和光心均保持原对称定义。右侧 180° 实体滚转、正网格尺度和右 optical X/Y 方向全部保留，没有像素翻转。

## 实际验证和边界

| 检查 | 本轮结果 |
| --- | --- |
| 原网格点/面身份 | 与 M4 USD 对照，全部 54 个 mesh 点、face counts、face indices 哈希一致 |
| 显示接口 | 54 个 mesh 角点法线；4 个 arm mesh 连续 UV；2 个 palm 单一银白材质；1 个相对路径色带及哈希正确 |
| 针对性 CPU USD 测试 | 4/4 通过：平滑接缝保点面、锐边保持、掌壳材质、UV/法线/纹理链 |
| 当前姿态相机 visual AABB | 两侧均与其余已提供 arm/mount/hand visual AABB 分离 |
| 当前姿态 315 条自遮挡射线 | 左由 18/315 变为 **20/315**；右由 20/315 变为 **16/315**；两侧中心射线畅通 |
| 实际 Blender CPU Cycles | 最终 `blender02` 导入和 7 个审阅视图完成，真实纹理已打包；Astra 和主会话实际看图 |

射线仍采用既有 synthetic 640×480、fx=fy=600 px、主点 `[320,240]`、21×15 采样及 optical Z=0.01～0.5 m。比例不是精确像素遮挡率。AABB 分离只覆盖本姿态和已提供 visual，缺失的支架/线缆、PhysX collision proxy、动态全行程、接力目标覆盖不在本轮通过范围。

首个 FK 核对把组合 URDF 的约 4.9e-12 旋转序列化残差误当作失败；容差按其有效位数改为 1e-10 后重跑全部两侧几何及射线检查。显示报告检查亦修正了把 palm 子孙指节误匹配为 palm 的路径条件，最终限定实际 palm link 后全量通过。这些中间尝试没有用于验收。

## 私有证据与命令

ignored `.local/evidence/m5/astra/` 保存 `approved_observation.json`、源截图哈希、`outward_geometry_review.json`、`usd_validation.json`、`COMMANDS.md`、`preview01` 和 `blender02`。主会话的 `.local/evidence/m5/delivery/` 提供 `wrist_before_after.png`、`outward_camera_front.png`，包含旧/新腕部、中心标记、20 mm 箭头及静态范围图例；本 Astra 已实际查看。用户截图及第三方派生图不随公开提交发布。

准确机器命令和各次结果记录在私有 `COMMANDS.md`；实际运行的核心入口为：

```text
scripts/assemble_commissioning.py --profile ... --binding ... --out ...
scripts/build_commissioning_preview.py --profile ... --left ... --right ... --table-usd ... --appearance ... --out ...
python -m unittest discover -s tests -p test_preview_appearance.py -v
review_outward.py
validate_usd.py
blender --background --python scripts/blender_review_scene.py -- --usd ... --out ... --wrist-closeups --symmetry-views --material-mode reference
```

复用既有独立 CPU 环境、现有官方资产和只读 NAS；没有新增大依赖、启动 GPU、驱动真机或改碰撞。未运行动态抓取、物理 RGB、标定板投影、全行程视野或 LeRobot 导出。剩余阻塞仍为相机支架与紧固、完整机械配合、生产惯性、实测标定和运行时接触验收。
