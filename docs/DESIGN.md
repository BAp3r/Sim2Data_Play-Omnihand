# Sim2Data 设计基线 v0.3

2026-09-23 运行时补充：完整装配 USD 的静态 RTX smoke 与物理/驱动验收分开。场景加载和三路 RGB 只能证明视觉层可渲染；若 schema inventory 没有机器人 rigid body/articulation，不能进入接触任务。Kit 关闭退出码和 shutdown 栈单独记录，访问冲突不得记为正常关闭。

用户随后授权一条规划运动样本：Pinocchio 使用真实 URDF 求解，Isaac 逐帧运动学回放并采集三路 RTX，官方 LeRobot 写入一 episode。该样本的 state/action 是规划配置及下一帧配置，不是真实反馈/控制命令；独立 synthetic 标记与 `task_success=null` 保持接触任务和生产门禁，详见 `MOTION_SAMPLE.md`。

状态：**设计基线及分项实施，未通过机器人生产验收**。更新：2026-09-22。用户已恢复 A/B/D 和有条件的运行时 smoke；本轮不做接触任务 C 或批量采集。资产、装配、运行时和 SDK 小样分别记证，不互相替代。已运行范围见 `docs/VALIDATION.md`。

## 1. 技术选择

主线建议为 **Isaac Lab + Isaac Sim / PhysX + USD + Blender 补充建模**。已有官方 Isaac 资产、本任务强调多视角视觉和域随机化、用户要求 USD/LFS，这些约束使直接沿用 USD 渲染与物理链路更合算。mjlab 保留为后续可选物理后端，不在第一阶段承担两套资产转换、材质迁移和接触差异的维护成本。

mjlab 不是不能并行：其官方项目结合 Isaac Lab 风格的 manager API 与 MuJoCo Warp GPU 物理。这里选择 Isaac Lab 的原因是现有资产和视觉管线，不是声称 MuJoCo 不能批量仿真。

第一轮不同时实现两套 backend，只固定后端中立的 robot manifest、episode/timestamp/schema 和资产索引。规划求解器先用简单可调试的 IK + 可行性/碰撞检查；需要时再接入更强的规划器，而非安装阶段一次引入所有重依赖。

## 2. 工程分层

```text
已校验官方模型 + 自建转接件/相机支架 + 实测标定
                         ↓
                 USD 场景/机器人装配
                         ↓
物体与目标采样 → 可达/无穿透过滤 → 接触抓取专家/任务状态机
                         ↓
                 物理状态 + 同步相机帧
                         ↓
               成功判定与质量门禁
                         ↓
     有界队列 / 分片写入 → LeRobot v3 官方 writer
                         ↓
        finalize → loader 回读 → 原子完成标记
```

机器人、任务、采样、相机、导出、批处理独立模块；不要写成一个同时启动 Kit、控制手指、编码视频和管理 Git 的巨大脚本。

源码已拆分为 `packages/sim2data_core`（数据契约、预检、资产接口）、`packages/sim2data_isaac`（坐标链与装配需求）和 `packages/sim2data_lerobot`（官方 SDK writer/loader 适配）。保留 `sim2data.*` 导入接口，具体安装方式见 `PACKAGING.md`。物理 collector、接触控制器和 Mimic 适配尚未实现。官方上游以固定 Git submodule 纳入，两套 Linux 运行环境各有真实 uv 锁；锁定不等于运行验收。

## 3. 机器人与坐标契约

桌面中心表面为世界原点，x 向操作者右方，y 远离操作者，z 向上；使用米、弧度和右手坐标系。`T_A_B` 表示把 B 中的列向量变换到 A。项目序列化四元数采用 wxyz，各第三方适配器显式转换，禁止凭库名猜顺序。

每套机器人有以下独立 frame：

```text
world → arm_base → ... → arm_flange → mount_assembly → hand_root → finger_links
                                                  └→ camera_housing → optical_frames
hand_root → grasp_tcp（明确指端/掌心关系，不等于任意 URDF root）
```

例如下置相机：

`T_world_optical(t) = T_world_flange(q(t)) × T_flange_mount × T_mount_camera_housing × T_housing_optical`。

草案保存的原子变换为每侧 `robots.<side>.T_flange_mount`、`T_mount_hand_root`，以及 `camera_candidates.wrist_<side>.T_mount_camera_housing`、`T_housing_color_optical`。每只手独立绑定 `hand_asset`。变换值使用 `translation_m` 三向量与 `rotation_wxyz` 单位四元数；`T_flange_hand_root` 和 `T_parent_color_optical` 是可选派生值，不能替代缺失的原子链。未来装配器须由链重新计算并核对派生值，当前预检仅查原子链声明是否缺失。

4 cm 只是用户提供的名义长度，不确定平移轴方向，也不提供旋转；不能用它替代手眼标定。相机外壳的物理安装与其 optical frame 必须分开。OpenCV optical 约定、USD camera 约定和 Isaac Lab 相机配置通过适配器转换，并用标定板投影测试确认方向。

只用与真实 AIRBOT Play 版本匹配的官方运动学、关节限位和惯性。OmniHand 必须核对普通/Pro/硬件修订及左右手；官方站点分别提供左右 URDF，不要对一个右手模型做负比例镜像来冒充左手。

用户提供的 AIRBOT 候选为 [ROS2 模型集合中的 airbot_play.urdf](https://github.com/foiegreis/ros2_quadrupeds_and_humanoids/blob/main/manipulator/Airbot/airbot_play_description/urdf/airbot_play.urdf) 和 [DISCOVERSE 的 airbot_play_v3_gripper_fixed.urdf](https://github.com/discoverse-dev/DISCOVERSE/blob/main/models/urdf/airbot_play_v3_gripper_fixed.urdf)。它们目前仅是来源线索，尚未绑定为生产模型；需要审阅来源 commit、许可、实物修订、惯性/限位和 mesh 依赖，并识别原夹爪与机械臂法兰的连接边界，再装配 OmniHand。

导入后验证：主动电机通道、物理关节、mimic/耦合映射、角度正负、零位和驱动单位；再核对 actuator effort、速度、增益、阻尼、接触碰撞近似与自碰撞过滤。URDF 可显示不代表指尖接触可用。导入器折叠固定关节后，应检查 frame 绑定是否仍正确。

转接件和相机总成计入载荷与惯性。只有明确焊接刚体关系后才能合并刚体，且须保留查询用的坐标框架。Blender 只补支架、转接件、桌面等，不靠外观重画官方机械臂来替代运动学。

## 4. 相机和场景匹配

用户已确认两只腕部 D405，主相机允许采用通用仿真相机，因此 RGB 名称固定为 overhead、wrist_left、wrist_right；实际腕相机流配置和标定仍需冻结。主相机不是手机绕拍相机；本视频的 1440×1080 展示尺寸不对应任何训练流。

| 流 | 启用状态 | 参数来源与阻断条件 |
|---|---|---|
| overhead RGB | 必选 | 640×480 / 30 Hz、K 为 synthetic 设计；世界位姿仍必填，随后验证覆盖范围 |
| wrist_left / wrist_right RGB | 两路必选 | D405 由用户确认；各自分辨率、K、畸变、流配置和原子光学链待补 |
| 任一 depth | 当前关闭 | `dataset.depth_enabled=false`；开启需单独扩展标定、单位、存储和读取契约，当前预检会拒绝直接翻开此开关 |

允许任意通用主相机型号，不表示允许缺失它的位姿。视觉审查的批准观察包由主会话/Astra 提供；Luna 消费观察 ID、帧/时间位置、结论和不可推断量，完成确定性接口或几何工作。

为每个 RGB/depth 流记录实际分辨率、K、畸变模型、裁剪/resize、光学外参和 frame 时间。先做理想渲染，再根据真实流加有限的噪声、曝光、白平衡、模糊、畸变或深度缺失；不要声称理想深度就是 D405 输出。畸变/resize 改动必须同步改变标定描述。

V1 优先 RGB；物理真值深度和模拟传感器深度分别标识，只有确有下游需求才批量保存，避免无谓 I/O。深度用无损 uint16/float32 外部存储并记录单位，不能塞入有损 RGB MP4。当下游需要原生 LeRobot 深度特征时，单独建立版本化扩展及读取测试。

桌面、底座安装板、双臂、下置相机支架、目标外形和视锥内显著背景优先复现。支架/手腕造成的遮挡不能在渲染时隐藏。第一阶段不用精确模拟工作区外杂乱电缆。

## 5. 双臂桌面中转任务（2026-09-22 用户确认）

正式任务固定为：左臂从桌面左侧取盒，放到桌面上两臂共同可达的中转区；左手松开并撤离，物体稳定受桌面支撑后，右臂重新抓起同一个物体，放入桌外右侧框子。用户选择了桌面中转。两只手下均安装 D405；主相机允许采用通用仿真相机，其设计参数应标为 synthetic，不冒充实测标定。

开发时先独立验证左臂的取放段和右臂的取放段，再串成完整回合。正式数据中的双臂 state/action 均需完整记录，非活动臂保持安全姿态，始终参与渲染与必要碰撞。框子的内部尺寸、边框碰撞、底面支撑高度、桌边距离与右臂可达性需明确验证；“桌外”不意味着框子底面与桌面等高。

任务先采用完整的可解释状态机：

`reset/settle → sample → pregrasp → approach → finger close → lift check → transfer → lower → release → retreat → stable-place check`。

上述单段流程执行两次。第一段 stable-place 的目标是桌面中转区，必须确认左手接触解除、物体稳定、左臂撤离后才允许右臂 approach。第二段目标是外侧框子，最终验收要求物体被框内有效区域容纳并稳定，不能只检查物体中心越过框口。中转失败、二次抓取失败、碰框或掉落分别记录；整条双臂链成功才计为成功 episode。

配置显式包含 `relay_stability_and_clearance_check` 阶段与 `task.relay_gate`：同一物体、左手接触解除、桌面支撑、完整 footprint 在中转区、速度满足稳定驻留条件、左臂避开右臂接近路径。线/角速度阈值、稳定/松手驻留、最小净空和超时尚为 null，必须阻断正式采集。每次判定的物体 ID、仿真步、接触/支撑/区域判断、速度、净空、已驻留时间及失败原因写入 sidecar。当前只冻结判定接口，不实现或假装通过物理接触门禁。

专家可以使用物体真实位姿规划、采样抓取方向和预抓手型；这些是 privileged 信息，不能直接混入未来只看相机和本体感觉的策略 observation。对于简单盒状物，先做有限侧抓/包络抓候选，而不是要求 RL 从零发现所有手型。

手指不是一个标量夹爪：预抓手型和收拢轨迹最终必须展开成真实设备支持的独立电机控制量。记录最接近真实接口、经限幅/耦合后的命令，同时保留生成该命令的 TCP/手型意图供调试。未驱动的被动关节若真机不可观测，只能作为仿真 sidecar，不应默认进入可部署 observation。

严禁通过持续设置目标位置、物体粘手、关掉目标碰撞、抓取瞬间清零重力等办法生产“成功数据”。失败是正常输出，要归类，而不是改成功判据掩盖。

目标成功至少满足：已从初始支撑抬离、移动到目标支撑区、物体 footprint 而非仅中心位于有效区域、已松手、物体不再依附手、速度和姿态稳定持续一定时长、手撤离且没有非法碰撞。相应高度、速度、容差和驻留时间作为经过验证的配置，不在当前未测尺寸时写成真实值。

## 6. 域随机化

先固定标定做 nominal 回合，再增加随机化。训练/验证按 object identity、外观、seed 和场景参数分割，不能把同一物理 episode 的不同相机或不同渲染变体分到不同集合。

建议顺序：

1. 目标 XY/yaw 和可达的放置目标；尺寸围绕实测值做小幅变化，逐步扩大。
2. 目标印刷/纸色、粗糙度、细微折痕、桌面材质、光照与相机颜色响应。
3. 质量、摩擦、质心扰动、关节阻尼、执行延迟和传感噪声，在真实测量允许范围内变化。
4. 经标定不确定度约束的相机/装配外参扰动，而不是大幅改变固定实验装置。

尺寸随机化必须同步改变 visual/collision 尺寸、质量/惯性假设和抓取候选。粗糙度不等于物理摩擦；质量不等于密度，纸盒/软包也未必是均匀实心块。需要记录采用的近似。包装是否可压缩先向用户核实，V1 刚体模型不能宣称覆盖显著形变。

每回合保存 base seed、episode ID 和 geometry/material/lighting/physics 子 seed 及实际采样值。seed 不依赖 worker ID，支持不同 worker 数量下重新分配任务；这不保证 GPU PhysX 位级确定性。

## 7. LeRobotDataset v3.0 数据契约

这里的 v3.0 是**数据格式版本**，不是要求安装名字为 `lerobot==3.0` 的包。采用支持该格式的官方 SDK，并锁定实际兼容的包版本或 commit。SDK 的元数据和分片格式交给官方 writer，不手拼一个看起来相似的目录冒充通过验证。

主要特征：

- `observation.images.overhead` 和已确认存在的腕相机 RGB 流。
- `observation.state`：具名、单位明确、真机可观测的本体反馈向量。
- `action`：当前观察之后将执行的硬件可行命令向量。
- 官方索引、episode、task、timestamp 等由 SDK/适配器按固定 schema 管理。

不能先假定 action 长度就是 USD 中非固定关节数量。若每只臂 6 轴、两只手各有 nL/nR 个独立控制通道，关节位置式 action 的候选长度为 `12+nL+nR`；最终还取决于真实控制模式和接口。场景左右和解剖手型不要混淆。所有索引映射冻结进 robot manifest，并做单关节/单电机回归测试。

时序初值建议 physics 240 Hz、control/RGB 30 Hz，8 个 physics tick 对一个记录帧；这是基准测试起点，不是已证实的稳定步长或实际相机频率。记录的是仿真时间，不是渲染墙钟时间。每个 tick 先获得 s[t] 和 I[t]，随后记录 a[t] 并执行；不能把渲染器滞后一帧的图像标成当前状态。停稳初始化可发生在 episode 开始前，但必须用相对时钟对齐并保留原始偏移。

具名 state/action 通道在创建布局时复制为不可变序列，调用方随后修改原列表不能改变数据维度。时间戳必须是有限数值，布尔值、数字字符串、NaN/Inf 均拒绝。当前本地测试只验证声明的时钟标签，渲染器实际图像延迟仍需后续测量。

sidecar 保存：物体位姿、接触、成功/失败原因、FK TCP、每路 K/T、实际随机化值、物理引擎/驱动/模型版本、资产哈希、代码 commit、episode UUID。它们默认不作为视觉策略输入。depth 和稠密接触按需保存，以控制体积。

writer 的完成流程必须包含 `save_episode()`、`finalize()`、重新打开官方 loader 验证 frame 数、视频边界、统计量和时间窗口。只有回读和 QA 通过才发布完成标记；异常分片不得继续当作有效训练集。若只保留成功数据，要先缓冲一个有界 episode 或写到可清理暂存区，不能保存后粗暴删除表格行而不更新视频偏移和统计。

## 8. 批量化与性能

先跑通 1 个环境，再测 4/8/16 等递增并行度；不凭“PRO6000”型号就承诺某个吞吐。需记录完整 GPU 型号/显存、驱动、仿真版本、渲染设置以及冷启动/预热差别。

以有效成功 episodes/hour 和 GB/episode 为主指标，同时拆分 physics、渲染、GPU→CPU、编码和写盘时间、峰值显存/内存、队列长度、失败原因。多个环境的相机必须空间隔离，不能让邻接环境出现在画面、阴影或反射中。

初期同一次仿真内记录数据，先证明没有相机延迟和漏帧。之后才优化为“无相机物理验证 → 保存完整可回放轨迹 → 分批渲染成功轨迹”。后者要记录所有可见 articulation/object 的时间对齐状态；渲染回放仅重现已验证物理状态，不再重新积分以免轨迹漂移，也不得用这种 kinematic 回放伪造物理成功。

禁止多个进程同时向同一个 LeRobot 根目录/同一 writer 写入。首版一个协调 writer 接收多个环境数据；需要多 writer 时每个 worker 写独立合法 shard，再用经当前版本验证的合并流程重建全局索引和统计。每个 worker 使用不同临时目录，完成后原子转入 ready。崩溃重启按 episode UUID 和完成清单去重。

## 9. uv、NAS 与 USD 存储

仿真运行环境和 LeRobot 导出环境分开，用各自 pyproject/uv.lock 锁定软件版本，防止 Isaac Kit 的 PyTorch/CUDA 约束和导出环境相互覆盖。当前官方 Isaac Lab pip 文档示例为 Isaac Sim 5.1 / Python 3.11；最终版本由服务器已有安装、资产包版本和实际 headless 相机启动测试决定。不能在没有清点时重新下载整套环境。

代码、uv cache、venv、Kit shader cache 和正在写的数据放服务器本地可写 NVMe；大型官方资产保持在 NAS 的只读挂载。项目 `assets/external/` 可软链接到该挂载，但 Linux 软链接不能代替 SMB/NFS 挂载。

uv cache 和 venv 尽量在同一文件系统，用 hardlink 或受支持的 reflink 来减少重复占用；不建议让 venv 包软链接到可清理缓存。不要 `sudo uv` 或把多用户可写缓存无隔离共用。保持 per-user 可写目录，热数据完成后按 shard 迁移到 NAS。

不能将 `@${SIM2DATA_ASSET_ROOT}/...@` 写进 USD 并假定自动 shell 展开。先由 Python/配置解析实际路径，再创建本机 ignored override layer 或受控 resolver。保持供应商资产相对依赖目录结构；展开 USD、MDL、纹理和 payload 的依赖闭包，发现远程 URL/缺失依赖时显式报告，而非默认联网拉取。

只对选用依赖/自建产物计算并记录哈希，不对 NAS 官方资产全集反复哈希或复制。官方资产是否可再分发按许可单独审核，项目 MIT 等代码许可不覆盖供应商资产。

## 10. Git 与 LFS

自建/经授权纳入版本库的 USD/USDA/USDC/USDZ 走 LFS；本包 .gitattributes 已写好。外部官方全集、数据集、真实录像、缓存、机器路径和凭据不进入代码仓库。公有 GitHub 中尤其不能混入实验室实录和内网部署文件。

内网远程与 GitHub 的主从/镜像策略尚待确认，暂不覆盖任一 main、不 force push、不将两个独立历史强行关联。建议私有内网仓库存完整可授权项目资产，公开镜像只同步经审核的代码和文档；若需要保留完全相同的 Git 历史，应改用私有镜像/独立私有资产仓库，而不是误以为 Git 分支能隐藏公开历史中的私有数据。

Git over SSH 可用不等于 LFS 端点已可用。实际要测试一个小的自建 USD：提交、上传 LFS 对象、全新位置拉取、确认不是 pointer、打开组合场景。不要用海量 `lfs push --all` 作为第一步。

## 11. 里程碑与退出条件

| 阶段 | 工作 | 验收 |
|---|---|---|
| M0，本包 | 证据/设计、只读盘点、基础契约与测试 | 纯 Python 测试通过；未知参数明确为空，不声称仿真完成 |
| M1 | 服务器/官方资产/机器人版本清点，绑定本地路径，锁环境 | 不重复下载；headless physics+RGB 最小场景；card_box 及依赖可打开；LFS 新克隆可读 |
| M2 | 双臂、手、转接/下置相机和桌面装配 | 单通道运动映射正确；无初始穿透；相机标定/投影/遮挡检查；待测值消除 |
| M3 | 单环境真实接触 pick-and-place | 多个初始姿态可成功；失败原因可解释；无粘手/瞬移；动作受硬件限位约束 |
| M4 | 少量有效 LeRobot v3 回合 | 官方 loader 回读、视频时间/索引/统计一致；抽看所有流；基础训练 batch 可取出 |
| M5 | 受控随机化、并行度扫描、断点恢复 | 分层成功率报告、无串景/串帧、重复 ID 检查、有效吞吐和存储测量 |
| M6 | 大规模正式采集与下游验证 | 固定 manifest/版本/划分，抽检和覆盖统计；真实小验证集评估 sim-to-real，不只看合成效果 |

优先级是 **真实装配与观测/控制对齐 → 接触成功 → 数据正确 → 随机化覆盖 → 吞吐**。

## 12. 参考来源（2026-09-21 核验）

以下是技术文档来源，不代表已在用户服务器复现。

- LeRobotDataset v3: https://huggingface.co/docs/lerobot/lerobot-dataset-v3
- Isaac Lab pip/uv installation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html
- mjlab official repository: https://github.com/mujocolab/mjlab
- AIRBOT Play official documentation: https://docs.discover-robotics.com/en/airbot-play/quick-start/overview.html
- OmniHand official model/SDK downloads: https://www.agibot.com.cn/DOCS/OS/Omnihand-O10
- uv cache: https://docs.astral.sh/uv/concepts/cache/
- Git LFS: https://docs.github.com/en/repositories/working-with-files/managing-large-files/about-git-large-file-storage
- OpenAI custom subagents: https://developers.openai.com/codex/multi-agent

## Synthetic 装配调试边界（2026-09-22）

用户已允许独立 synthetic 场景和安装假设。`configs/commissioning.synthetic.json` 与生产草案分离，生产变换继续为 null；真实网格静态 USD/Blender 预览不进入数据导出。D405 采用固定官方 ROS 几何和 screw→optical 名义内部链，外部安装、质量惯性和实物标定仍待核。旧夹爪座残留已由替换用户单臂模型解决；机械搭接和关节限位内的相机覆盖未解决前，不通过 A/B 接触门禁。

装配修正补充：使用用户提供的 `play` 六轴包替换含旧夹爪座的候选，参考 `ASSEMBLY_CORRECTION.md`。保留J6及其原生输出件；独立安装候选采用原生link6坐标，不能继承旧候选的bbox偏置。Thor table 使用原生0.9×0.7586 m桌面，视觉平面z=0与碰撞上界z=-0.0155 m分别记录；此差异未解决前不得宣称桌面接触通过。

左右D405对称设计补充：保持真实相机网格和正尺度，通过右相机绕光轴180°及screw基准平移补偿，使对应mount坐标中的外壳中心与双目光心对称；右optical X/Y随实体滚转，禁止隐式像素翻转。对称关系不等于完整机壳/孔位镜像，支架与生产外参仍待核。详见 `WRIST_SYMMETRY_REVIEW.md`。

用户追加修正：两台D405分别向场景外侧移动20 mm，保持相机刚体旋转和官方housing→optical链。该要求定义synthetic位置增量，不确认实物安装外参。用户实录优先约束本次外观：连续银白掌壳与平滑浅色臂侧板；材质修正不授权改动源网格几何。

NAS仓库纸盒headless调试采用显式synthetic wrapper：均匀缩放0.12（约0.084×0.06×0.06 m）、质量0.08 kg，惯性由PhysX依据已有碰撞与该质量推导。保持源boundingCube碰撞，不写回源资产。这些值不是实测桌面目标，不冻结生产物理模型。该smoke的桌面为合成静态碰撞平台，尚非双臂/Thor接触验收。


## 2026-09-23 夹爪控制接口与桌外框高度

本轮新增 `sim2data.control.gripper.GripperMap` 作为合成调试接口。输入一个 `[0,1]` 开合量，分别展开为左右 OmniHand 各 10 个独立主动关节目标；6 个 URDF mimic 关节不写命令，由 PhysX articulation 耦合。左右映射、符号和开合端点独立声明，不能镜像复制。驱动刚度、阻尼、力和速度只是 commissioning 假设，必须在无物体响应试验中核对。

外侧框子下一版 synthetic 布局将其有效底面/支撑顶面设为与 Thor 桌面顶面同高；框壁仍有碰撞厚度，物体必须落在框内底面，不能用穿透隐藏高度差。这个布局值不等于实测硬件尺寸。

场景地面采用独立连续 flat plane（`/World/FullFlatGround`），位置低于 Thor 桌体底面；它不是用来替代桌面，也不通过巨大 Cube 的厚度制造支撑。

2026-09-24 M10 更新：独立 fresh USD 的左右 SingleArticulation 已初始化并各执行480步空载驱动，但整体响应门禁失败（左侧目标裁剪归零、两侧四指mimic残差超限，inventory无collider）。这仅推进运行时诊断，不表示完成装配、抓取或接力验收。接触录制保持阻断；详见 VALIDATION.md。

2026-09-24 M11：M10的“缺collider”已纠正为instance proxy遍历漏报，两侧各25个collider。左右synthetic空载响应通过后已开始独立fixture的单臂真实接触调试，首次两条都失败且无成功视频。使用当前源URDF坐标，旧硬件手势仅作意图参考；保持原始生产参数null和采集关闭。详见VALIDATION.md最新条目。

M11当前接触门禁仍失败：contact02有真实接触但推倒盒子，contact03数值失稳且未通过保持判据。新增稳定性界限，防止异常速度被误判为lift。

M11最终对照：低力、1 ms的contact07完成20秒且未触发状态界限，但仍推倒盒子，连续接触抬升保持为0步。空载关节响应和接触力已有读回证据，成功抓取和有效闭合视频仍缺。源机器人惯性未覆盖；视觉与动力学验收分开。

2026-09-24 M12：全局真值规划已实现真实 URDF FK/mimic、接触几何拟合和 IK；候选仍因碰撞门禁失败而拒绝执行。接触场景统一为官方 Thor + 连续 Mesh 地面 + 0.12 倍官方 cardbox，候选资产独立入库，不自动替换。详见 GLOBAL_STATE_CONTACT.md。生产参数与接力门禁保持不变。
