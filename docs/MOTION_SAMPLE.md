# 一条 Pinocchio → Isaac RTX → LeRobot 规划运动样本

2026-09-23 用户明确选择先完成规划运动样本。范围为 **synthetic 运动学回放**；不代表 PhysX 驱动跟踪、无碰撞路径、抓取或完整双臂接力。生产采集继续关闭，`task_success=null`。

## 实际输入与规划

输入仍是审阅过的用户 play 六轴 URDF、独立左右 OmniHand、D405 与 Thor 装配。arm URDF SHA256 为 `b02c7ac6fd1dd6a65f355ef6650c2a7584fdbd56ccfc497b767292010ce4e0b0`，synthetic profile SHA256 为 `40d96e8fbb7a9745792d646c17c442ed28737717eaa5d5dc403be9ff79f3d33b`。私有机器路径只保存在 `.local`。

`scripts/plan_kinematic_motion.py` 实际使用现有 CPU 环境的 **Pinocchio 4.0.0**，从该 URDF 构建模型。位置 Jacobian 的阻尼最小二乘 IK 将左 link6 目标上移 25 mm，再用 Pinocchio `interpolate` 和五次时间曲线生成往返轨迹。右臂保持原姿态，双手保持审阅过的张开姿态。这里没有使用 cuRobo，也没有将 IK/插值称为避障规划。

位置 IK 4 次迭代，终点残差约 `8.0e-8 m`；121 个配置，30 Hz，首末时间戳 0～4 秒。有限差分峰值速度约 `0.08311 rad/s`，小于本次明确的 synthetic 上限 `0.25 rad/s`。源 URDF 的零 effort/velocity 未被篡改为生产参数。固定姿态的首个 IK 候选超过该速度上限，已拒绝，没有采集。

## 实际场景采集

`scripts/isaac_scene_smoke.py --trajectory <private-plan.json>` 调用 `scripts/isaac_motion_capture.py`。先核对初始 Pinocchio FK 与审阅 USD 的每级 arm 变换；每帧更新六轴对应的局部 USD 变换，固定的手、法兰和腕相机随父节点运动。每帧再核对世界腕部变换，实测最大矩阵元素差 `7.77e-16`。

所有流在同一次 FK 更新后以 RTX 渲染，获得 121×3 张真实 `320×240` RGB。没有复制单张静态图来充当运动视频；纸盒保持原位，没有粘手或抓取声明。相机与轨迹共用显式回放时钟，PhysX 没有推进，因此不能声称动态传感器延迟或物理同步已验证。

证据为 ignored `.local/evidence/m8/trajectory02.json`、`capture02/capture.json`、`capture02/images/`。主会话实际查看起点、最高点和腕部图；腕相机仍主要看到指尖/背景，当前运动不是任务视野验收。带图例的起点/最高点对照及三路 GIF 位于 `.local/evidence/m8/delivery/`，不公开第三方派生图。

## 数据语义与导出接口

`scripts/export_motion_sample.py` 消费 capture manifest 和逐帧 PNG，用现有 LeRobot v3 官方 writer 保存恰好一个 episode、finalize，然后官方 loader 回读和取训练 batch。十二维 state 为两臂用于 FK 的规划配置；十二维 action 为下一帧规划配置，最后一帧保持。它们不是电机反馈或硬件命令。手部姿态固定的限制保存在 sidecar。

现有适配器的 `i*8` 时钟标签只用于格式接口，sidecar 明确说明其为 synthetic adapter timeline，不表示发生了 240 Hz 物理步进。成功与物理字段不会被升级；SDK 生成正式格式元数据，不手拼元数据。

已实际完成官方 LeRobot 0.6.2（固定 commit `b64fe1ed9f11eeac53ee821356d2797601701054`）写入、finalize、回读与训练 batch：1 episode / 121帧；三路 MP4 各121帧。全部帧的 state/action 误差小于 `3e-8`，时间戳误差小于 `1.2e-7 s`，编码图像与输入逐帧平均像素误差均小于5/255。最终带每帧相机位姿的结果已下载到 `.local/evidence/m8/dataset03/`，压缩包 `.local/evidence/m8/exported03.tar.gz`，回读报告为 `full_readback03.json`；未上传 Hub 或公开 Git。SDK 使用先前已审核的私有 overlay，未修改共享环境。

## 仍未通过

完整清理两次定位到 Kit 扩展卸载访问冲突，运动采集后也以 `0xC0000005` 退出。图像和 manifest 在异常前已经写完；单独的 `process_exit.json` 明确记录非正常退出，未杀他人进程或跳过清理冒充成功。物理驱动、碰撞、生产外参、机械配合与惯性、Thor 约15.5 mm支撑差、接力 C 与批量采集仍未验收。
