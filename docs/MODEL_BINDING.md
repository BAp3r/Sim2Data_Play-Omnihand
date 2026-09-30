# M2 model binding audit

状态：**静态来源与包闭包可审阅，物理绑定未通过**。更新：2026-09-22。
本记录只绑定来源、URDF 结构和私有包内的资源闭包。它不证明硬件型号、装配标定、Isaac Sim/PhysX 碰撞或抓取行为，`production_collection_allowed` 继续为 `false`。

## 输入与证据

本轮使用的输入是正式发布历史 `335a747` 的工作树、既有 A 审计的两个 AIRBOT URDF 快照，以及官方 OmniHand O10 页面指向的左右 URDF/CAD 下载包。下载包和解析 JSON 都保存在 ignored 的 `.local/evidence/m2/`，没有复制到仓库或提交 Git。

| 对象 | 来源与静态证据 | 状态 |
| --- | --- | --- |
| AIRBOT Play ROS2 候选 | `https://github.com/foiegreis/ros2_quadrupeds_and_humanoids/blob/7792960fb60f3118d9827b641dcc441e9ae2d06f/manipulator/Airbot/airbot_play_description/urdf/airbot_play.urdf`；快照 `.local/worktrees/a-assets/reports/github_urdf/foiegreis_airbot_play.urdf`；SHA256 `08cce00b5be18ca3f828aae5f41a5acd7b7b0607f6a0fce27bd0994048772cc3` | 候选；Apache-2.0 仓库许可快照已保留，mesh 包和硬件修订未绑定 |
| AIRBOT Play v3 DISCOVERSE 候选 | `https://github.com/discoverse-dev/DISCOVERSE/blob/d67f47c084aba0e0cf422a8725235f8b9238655a/models/urdf/airbot_play_v3_gripper_fixed.urdf`；快照 `.local/worktrees/a-assets/reports/github_urdf/discoverse_airbot_play_v3_gripper_fixed.urdf`；SHA256 `dc4fe8b37a6668a4432fba698aa3ef8207fe31a00d74c587988f58cf0f7a4b55` | 候选；固定 commit 的 LICENSE 快照为 MIT（`.local/evidence/m2/discoverse_LICENSE_d67f47c_20260922.txt`，SHA256 `1ea6b7eb8deddd0e2b175d7e6b8f16a667b7065c4bce2e16d04014ac3aef0bff`）；URDF package mesh 未绑定 |
| DISCOVERSE 同 commit 的 arm OBJ | 固定 commit 的 `models/mjcf/manipulator/new_airbot_play/`；私有下载 `.local/evidence/m2/models/discoverse_new_airbot_play/` | `MJCF_VISUAL_CANDIDATE_ONLY`：7 个 arm link 对应 11 个 OBJ 已下载并哈希；它们不是 fixed URDF 声明的 STL package，不能直接混装 |
| OmniHand O10 左手 URDF | 官方页面 `https://www.agibot.com.cn/DOCS/OS/Omnihand-O10`；下载链接 `https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813527171066.zip`；`.local/evidence/m2/models/OmniHand2025_left_urdf.zip` | 官方链接与 ZIP 哈希已绑定；包闭合静态核验；硬件铭牌和装配未核验 |
| OmniHand O10 右手 URDF | 官方页面同上；下载链接 `https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813525152953.zip`；`.local/evidence/m2/models/OmniHand2025_right_urdf.zip` | 官方链接与 ZIP 哈希已绑定；包闭合静态核验；硬件铭牌和装配未核验 |
| OmniHand O10 左手 CAD | 官方页面同上；下载链接 `https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813704283815.zip`；`.local/evidence/m2/models/OmniHand2025_left_model.zip` | CAD-only：ZIP 只有一个 STEP 文件，没有 URDF 或仿真 mesh |
| OmniHand O10 右手 CAD | 官方页面同上；下载链接 `https://www.agibot.com.cn/file/ueditor/php/upload/file/20260827/1787813704317306.zip`；`.local/evidence/m2/models/OmniHand2025_right_model.zip` | CAD-only：ZIP 只有一个 STEP 文件，没有 URDF 或仿真 mesh |

官方页面和包的确切再分发许可仍需单独审阅；私有下载包不进入公开 Git。先前的私有 OmniHand URDF 快照不能代替下面已经哈希绑定的官方 ZIP。

## 选择状态

| 资产 | 当前状态 | 可以交给下游的事实 | 仍然阻断生产绑定的事项 |
| --- | --- | --- | --- |
| AIRBOT arm | `CANDIDATE_STATIC_SNAPSHOT_PHYSICAL_PENDING` | `link6` 是两个快照中的法兰候选；URDF 关节图、原夹爪边界和 mesh 引用已解析 | 真实 AIRBOT Play 修订、完整 package、驱动映射、零位/方向、质量惯量和运行时碰撞 |
| OmniHand O10 left | `STATIC_PACKAGE_CLOSED_HARDWARE_PENDING` | `OmniHandleft3.urdf`、`l_palm` 根和 18 个被引用 STL 在官方 ZIP 内可读 | O10/O12 及硬件修订、左手安装确认、法兰转接、主动电机映射和 PhysX 导入 |
| OmniHand O10 right | `STATIC_PACKAGE_CLOSED_HARDWARE_PENDING` | `OmniHandright4.urdf`、`R_palm` 根和 18 个被引用 STL 在官方 ZIP 内可读 | O10/O12 及硬件修订、右手安装确认、法兰转接、主动电机映射和 PhysX 导入 |
| `card_box` | 沿用 `docs/ASSET_AUDIT.md` 的静态候选 | 官方 Isaac 5.1 USD 路径和依赖记录已经存在 | 动态刚体质量/惯量、目标适用性、接触和抓取运行时验证 |

## AIRBOT 候选结构

解析器把 URDF 的 `parent`/`child` 元素的 `link` 属性作为连接关系；`nonfixed_joint_count` 仅是 URDF 结构统计，不是硬件 action 维度。

| 快照 | URDF 结构 | mesh 引用 | `link6` 下游边界 | 解析证据 |
| --- | --- | --- | --- | --- |
| ROS2 `Airbot Play` | 11 links、10 joints；8 non-fixed，其中 `gripper_joint1` mimic `gripper_joint`；2 fixed | 24 个不同引用、26 个 visual/collision 引用，全部缺失；引用包名为 `airbot_play_description` | `gripper_joint` → `left`，`gripper_joint1` → `right`，`joint_custom_end` → `gripper_center`；另有 `camera_joint` → `camera_link` | `.local/evidence/m2/airbot_foiegreis_inspection.json` |
| DISCOVERSE `airbot_play_v3_gripper` | 11 links、10 joints；6 revolute、4 fixed；没有 mimic | 9 个不同引用、18 个 visual/collision 引用，全部缺失；引用包名为 `airbot_play_v3_gripper` | `endleft`/`endright` 固定连接 `left`/`right`，偏移分别为 `[0, 0.04, 0]` 和 `[0, -0.04, 0]` m；`joint_custom_end` → `custom_end_link` | `.local/evidence/m2/airbot_discoverse_inspection.json` |

两个快照都把 `link6` 作为当前装配切口候选。原夹爪、`gripper_center`/`custom_end_link` 和 ROS2 快照中的原相机均属于被替换的下游结构；OmniHand 安装必须使用独立的 `link6 → mount → hand_root` 链。DISCOVERSE 文件名中的 `fixed` 只描述原夹爪固定关节，不能当作 OmniHand 接口已完成。两个 URDF 虽然含 inertial 元素，当前没有实物质量、质心、惯量或 PhysX 运行时证据。

固定 commit 的 tree 审计显示，fixed URDF 声明的 `models/meshes/airbot_play_v3_gripper/arm_base.STL`、`link1.STL` 到 `link6.STL` 七个路径全部不存在。该 commit 另有 `models/mjcf/manipulator/new_airbot_play/` 的 arm visual OBJ；它们按 link 分组为 7 个 link、共 11 个文件，来源与 fixed URDF 同仓库同 commit，但格式、路径和 MJCF 绑定不同。当前只将其提供给隔离的静态预览，等待几何/单位/坐标等价审阅后再决定是否转换为 USD；不得把它们填入 fixed URDF 的 `package://airbot_play_v3_gripper` 闭包。

该候选的配套 MJCF 源是 `models/mjcf/manipulator/new_airbot_play/mjx_airbot_play.xml`（11,448 bytes，Git blob SHA `952a43d4f9204994793539339c6ef667e43f1828`，私有原文件 SHA256 `4ec2974a9824d6f6d8f42f4b464ffd65faa6173eac37c4943e643774c9c97cf6`）。OBJ 的 `meshdir` 由该 MJCF 的 `assets` 约定提供；这个 XML 与 OBJ 必须作为一个候选包审阅。

### DISCOVERSE arm OBJ 候选

下表的 Git blob SHA 来自 `d67f47c084aba0e0cf422a8725235f8b9238655a` tree；文件 SHA256 是私有下载后的实际文件哈希。OBJ 不声明米制单位，`bounds_native` 也不能直接作为世界尺寸。

| 固定 commit 相对路径 | 字节 | Git blob SHA | 下载文件 SHA256 |
| --- | ---: | --- | --- |
| `models/mjcf/manipulator/new_airbot_play/arm_base_0.obj` | 62,028 | `868e3a60dbf88a4ac84c2a311bca48aa72698bac` | `a8f4b3bd86b7b42f484f80b0d8abaf05bc32c3e769874539cd66db045291f2bc` |
| `models/mjcf/manipulator/new_airbot_play/arm_base_1.obj` | 280,815 | `a431f5209d7ad7debf3a57a26f8e47e72eb4a0d9` | `ef0ef26d2cf82e2fa82f8147ef47d37bdc76957e0c0c227a63d15cf38bb18d8d` |
| `models/mjcf/manipulator/new_airbot_play/link1.obj` | 210,649 | `575957d3edc31848c3b547ca41dd727e0fcb220e` | `d448e9742660b9f96f3d9510f23f94c8079aa52a0697aa21756094fc18688c2a` |
| `models/mjcf/manipulator/new_airbot_play/link2_0.obj` | 346,145 | `bf2b82c6c6ccdd987b469b0b9ba328cd05566d43` | `fd147d883a2af6484ca8c4060eaf2d2a03d70e89598f367d143e15298b3a0717` |
| `models/mjcf/manipulator/new_airbot_play/link2_1.obj` | 318,086 | `aecdd42b9dc53d9dfb0d040fa2884357188e028b` | `b148bceb4e4277a7ce753ef904103df43302789f9b64d386fd24ded0232871b8` |
| `models/mjcf/manipulator/new_airbot_play/link3_0.obj` | 337,406 | `258b739f6ac32dba3ba8fdbff9e9c3d75330656d` | `125db0b6739af668565a20dcfff16f9aa8202388b83e419d75520a7cf1d58edd` |
| `models/mjcf/manipulator/new_airbot_play/link3_1.obj` | 105,310 | `9ddd044295ec2939cf6f120c4ee390b58e5b5097` | `962261b0c1713e575cbcf3dfb4833b589bb0c35f2ffd395398e8e297d9126c1b` |
| `models/mjcf/manipulator/new_airbot_play/link4.obj` | 292,135 | `5fce4388bf6b984a6b4910cde687b3d276041316` | `890ae19b7f80cd60ed44611e8be4da97f631acb44f5b9fee18668890c2c2603f` |
| `models/mjcf/manipulator/new_airbot_play/link5_0.obj` | 311,640 | `952c4608be80c01ee642184b5241d68fcbca4d4d` | `d1415fee7d55699d3c7650633e1e5f8d97d8d44396b26e1b9758706df207b85b` |
| `models/mjcf/manipulator/new_airbot_play/link5_1.obj` | 10,785 | `6b1b69ecbbb4a8d640e0481d48b355639d4671be` | `364754f401e71b72ede3148e49385a7571c5ad07d230ab9c6295d1a85c2dd010` |
| `models/mjcf/manipulator/new_airbot_play/link6.obj` | 464,182 | `871920f388251ce2f3341a470fe0e9b154ef19db` | `82b583ceb042427ed87afa714177a7377c17487689f9af5a03fd386f724b83d4` |

## 官方 O10 URDF 包

### 包和 URDF 元数据

| 侧 | ZIP（字节） | ZIP SHA256 | URDF member（字节） | URDF SHA256 | package 根 | hand root |
| --- | ---: | --- | --- | --- | --- | --- |
| left | 10,607,076 | `8764a84d911dd5d46ece309bb3712e71c23fedffd366e353fc737f74f7e7efdb` | `OmniHand2025left/urdf/OmniHandleft3.urdf`（26,875） | `8c33ee9e9e9ce08cb9e379618d7e06f07791bf65a159e36f6481dd4e4d75e468` | `OmniHand2025left` | `l_palm` |
| right | 7,609,996 | `f4b48bbc28085370300496eac59a32f3bac497cdb9dda2447156c727c41fef34` | `OmniHand2025right/urdf/OmniHandright4.urdf`（27,158） | `5acb477166ae7b1211d2a8378fd1593e59e7742aa26f4f4578c0ba01bda9e952` | `OmniHand2025right` | `R_palm` |

左包有 34 个 ZIP member：1 个 URDF 和 19 个 STL；右包有 33 个 member：1 个 URDF 和 18 个 STL。URDF 使用的 `package://OmniHandleft3/...` 与 `package://OmniHandright4/...` 由归档根目录解析到实际 member；导入器必须保留这个显式映射。左右名称和 `l_`/`R_` 前缀只支持文件/URDF 侧别线索，不能证明安装到实物上的解剖左右手。

两个 URDF 都是 18 links、17 revolute joints、6 mimic joints，没有 fixed joint。URDF 含质量与惯量元素，按文件字段求和的手部质量均为 0.5343 kg；这不是已经核实的硬件质量，也不包含 mount、转接件或 D405。没有 `<transmission>` 或硬件 actuator ID，因此不能从 17 个 non-fixed joints 推断 action 维度。

左手的耦合关系为：`l_thumb_pip_joint` → `l_thumb_mcp_joint`（1.33），`l_thumb_dip_joint` → `l_thumb_mcp_joint`（1.3），四个手指的 `*_dip_joint` → 对应 `*_pip_joint`（-1.14444444444444）。右手对应关系为拇指 1.33/1.3，四个手指的 `*_dip_joint` → 对应 `*_pip_joint`（1.14444444444444）。左手 `l_middle_abad_jonit` 拼写与右手 `R_middle_abad_joint` 均有零上下限和零 effort/velocity 字段；这类静态细节必须与确切硬件/SDK 版本复核。

官方 SDK API 记录（`.local/evidence/m1/API_PYTHON_O10_026740d_20260922.md`，commit `026740d9fdd8ba32b0605fa702a992b322076f1b`）给出 O10 的 10 个 active 值和 6 个 passive/mimic 值。该 API 顺序、URDF 的 17 个物理关节和任何 Isaac actuator 配置尚未完成逐项校准；本文件保留 `hardware_action_mapping: null` 的边界。

### 左手引用 mesh

下表是 URDF 实际引用的 18 个唯一 STL；每个文件在 visual 和 collision 中各出现一次，字节数是 ZIP 未压缩 member 大小。

| 相对路径 | 字节 | SHA256 |
| --- | ---: | --- |
| `OmniHand2025left/meshes/l_palm.STL` | 6,418,934 | `26e9c382697fb7553cf537e53fa4da88d60bbc88bd444728c6a59bdf23586d1e` |
| `OmniHand2025left/meshes/l_thumb_roll_link.STL` | 331,784 | `48beab881c9c1784c96862e0de6b70f40921513a31beefc24de7376c5a3ffed0` |
| `OmniHand2025left/meshes/l_thumb_abad_link.STL` | 492,984 | `f3d55fc289ff0169ffa4318e258e57c025345985ae063d263b2fbb0bfd109997` |
| `OmniHand2025left/meshes/l_thumb_mcp_link.STL` | 940,484 | `da66acf46db91107de1d045665f537ae9475f613cb6f63b9becb222d820a6871` |
| `OmniHand2025left/meshes/l_thumb_pip_link.STL` | 323,484 | `1d867e853f1e464a8baa95d87b900d1b11b57ca8a24f2aa627cd5537319ab19c` |
| `OmniHand2025left/meshes/l_thumb_dip_link.STL` | 668,284 | `0614b9429dac7554cc1403fea79bf1e8f162d4b21afedb767abfcee2705213bf` |
| `OmniHand2025left/meshes/l_index_abad_link.STL` | 736,184 | `2eed047da092a612b64780a324bb0b9e0ad143cb169a85db38e1770b3200709c` |
| `OmniHand2025left/meshes/l_index_pip_link.STL` | 538,184 | `f167a0b4603efd61812e6ef86c64e75489d5c30520defcafdeca2a4b64e7769e` |
| `OmniHand2025left/meshes/l_index_dip_link.STL` | 1,268,384 | `7c997c94ff977714ef6fde809c8a7fcf802c79dfdd069e9e9e5d4fa94a0416c5` |
| `OmniHand2025left/meshes/l_middle_abad_link.STL` | 736,184 | `dc35c4c2be9addb2e35a8f9772eef4f54473b6733bd0dd12308390cf3862f04e` |
| `OmniHand2025left/meshes/l_middle_pip_link.STL` | 538,184 | `7c42b8dac80cedfd307739d3974883de35c91a1de619f8f0c5666a9c89bea492` |
| `OmniHand2025left/meshes/l_middle_dip_link.STL` | 1,268,384 | `e423a2e4262bfeb1691d5a8777d1470ffd40aff04d2c395a672a9ce88ee7c380` |
| `OmniHand2025left/meshes/l_ring_abad_link.STL` | 736,184 | `44884b83c075bd5c8cee3b6efc6a9451273b2b629edbce674a227c4e77caeb84` |
| `OmniHand2025left/meshes/l_ring_pip_link.STL` | 530,384 | `fca51152ee961911a3bc47a4ca1d07a37a79525b017f138bc4818d2af7e4a440` |
| `OmniHand2025left/meshes/l_ring_dip_link.STL` | 1,268,384 | `35d52264a136d7868456da0dabf5a930a1798167345419cd64270ba0fe6d1f3a` |
| `OmniHand2025left/meshes/l_pinky_abad_link.STL` | 736,184 | `10685a16293df4429c636dfe5211a6916645022d9cad3039d29be0f353027693` |
| `OmniHand2025left/meshes/l_pinky_pip_link.STL` | 530,384 | `9ab568750550b35d86f5c5bb72d2e7b72445ec28decc8f2bda57aabf27501319` |
| `OmniHand2025left/meshes/l_pinky_dip_link.STL` | 1,268,384 | `a7ae6a750f8f38f42bcdb5d6d2446a3c63e6cc2b30387e393372ed09c206b461` |

`OmniHand2025left/meshes/l_palm_old_frame_backup.STL`（6,418,934 bytes，SHA256 `19e20698ed6dbc97cb651ff8b7f8088eca2ead555c849c26fedf990fbd484747`）存在于归档但没有被该 URDF 引用，不能作为当前手掌几何绑定。

### 右手引用 mesh

| 相对路径 | 字节 | SHA256 |
| --- | ---: | --- |
| `OmniHand2025right/meshes/R_palm.STL` | 3,860,384 | `ab29d60b8738f606a4f3a93961c2483f35850a7da19230c37e4a1f90bc02b145` |
| `OmniHand2025right/meshes/R_thumb_roll_link.STL` | 327,184 | `7e461ece5c6e5fff225df151305fb082643b360d2ca696e004f07c2eba979d20` |
| `OmniHand2025right/meshes/R_thumb_abad_link.STL` | 437,184 | `9d6e44be3e0bccf445b79930c227303b70841f7be512cbc0718909672aa80cb0` |
| `OmniHand2025right/meshes/R_thumb_mcp_link.STL` | 1,050,584 | `c3984931d6aa8fa2e23486e4dcafbbfacf850267089ce3780033322762891548` |
| `OmniHand2025right/meshes/R_thumb_pip_link.STL` | 313,184 | `d78ee60679d98a779671a873cf2120254839d17e50d5c11f656647b3a5b8e5f3` |
| `OmniHand2025right/meshes/R_thumb_dip_link.STL` | 863,984 | `22e858bb96e78e913f28635c8d9f481f2d1d5d92d7bd9cbb1e0ba4566d39d691` |
| `OmniHand2025right/meshes/R_index_abad_link.STL` | 736,184 | `48afa539742c3662f29905a4fe65aefe96b51747b1e4311fed8d099afcf2e6d7` |
| `OmniHand2025right/meshes/R_index_pip_link.STL` | 524,834 | `c126f48b5557f832e8275f01fef0752728c712589c0aaaab1e6a4ba07c5fd45c` |
| `OmniHand2025right/meshes/R_index_dip_link.STL` | 1,248,734 | `dfb4f9b0d9be10fa28d9792a42d17b72b4f3610699f94ba9e16b02bd409bf813` |
| `OmniHand2025right/meshes/R_middle_abad_link.STL` | 736,184 | `3b7fc67612497ab7c5cba0212a4bde8be9038f5801a516829c07de0bb2a5baf1` |
| `OmniHand2025right/meshes/R_middle_pip_link.STL` | 524,834 | `daee6924fc8061efecf0f2a1d551d395c3d638b26931bc8aec0020f7bc078aa0` |
| `OmniHand2025right/meshes/R_middle_dip_link.STL` | 1,248,734 | `ac6576f086fc310addab2cf289cec7554c2569cac554f5a41892d2e2985804a8` |
| `OmniHand2025right/meshes/R_ring_abad_link.STL` | 736,184 | `39ae1b09ef0b76914f04815b6a91e651c95293c100cd70522d23d7eff2268257` |
| `OmniHand2025right/meshes/R_ring_pip_link.STL` | 524,834 | `f488d19bbfddd91e833417bb6c5d7f583b75f293706a8b7fc291f39506ea3cfa` |
| `OmniHand2025right/meshes/R_ring_dip_link.STL` | 1,248,734 | `2cdcf809a3e121426f5a00f685a87a142905858a09fda88291aa59c19ee5080b` |
| `OmniHand2025right/meshes/R_pinky_abad_link.STL` | 736,184 | `3071fe813f2e053d2614de4b5c500be44303c0ccb6a790c852cce8eb8fcee4f7` |
| `OmniHand2025right/meshes/R_pinky_pip_link.STL` | 524,834 | `f9f7ff88ced1f3ca9e227ae8fd85f805a6736bdd6edc30691653fdf960ad107b` |
| `OmniHand2025right/meshes/R_pinky_dip_link.STL` | 1,248,734 | `91a98ddb11e132e9d2fad5449c8a073bb11ccb570956ffc631efa9e42ebdbea0` |

## CAD 包和许可证边界

| 包 | 字节 | SHA256 | 静态内容 |
| --- | ---: | --- | --- |
| `OmniHand2025_left_model.zip` | 46,279,147 | `9bfeda661dd24edf26f92bc939b89987dc495efefaf3e6515d0393f6d362f3e5` | 1 个 STEP 文件 |
| `OmniHand2025_right_model.zip` | 46,804,929 | `5c2733a56582e267361e777e4f02761638989f670bc09cf047eba5eb6a64622e` | 1 个 STEP 文件 |

CAD 包没有 URDF、STL/OBJ 仿真 mesh 或碰撞近似，不能当作 Isaac 运行时模型。它们可以在后续审阅转接件和法兰机械接口时使用，但必须与对应 URDF 包和硬件修订重新绑定。官方页面链接是来源证据，包的再分发许可状态仍为 `pending_review`。

## 装配接口和缺口

下游 B/C 应消费下面的显式链，而不是把 URDF 根或名义法兰长度当作完整变换：

```text
arm_base → ... → link6 (flange candidate)
                       ↓
                 mount assembly
                       ↓
              l_palm / R_palm (hand_root)
                       ↓
                 finger links

mount assembly → camera_housing → optical (D405)
```

`T_flange_mount`、`T_mount_hand_root`、`T_hand_root_grasp_tcp`、转接件质量/惯量、两只 D405 的 `T_mount_camera_housing` 与 `T_housing_optical` 目前均为 `null`。用户估计的约 4 cm 只能作为待测量名义输入，不能产生旋转或替代法兰基准。手部 URDF 没有法兰 link；`l_palm`/`R_palm` 是待接到 mount 的手根。

下游接口应保留以下字段，并在缺失时阻断采集：

```text
arm_asset.source_commit
arm_asset.urdf_sha256
arm_asset.flange_link = "link6"       # candidate, not measured datum
arm_asset.mesh_closure_status
hand_asset.archive_sha256
hand_asset.urdf_member
hand_asset.urdf_sha256
hand_asset.hand_root_link
hand_asset.referenced_meshes
hand_asset.mimic_joints
hand_asset.hardware_action_mapping = null
assembly.T_flange_mount = null
assembly.T_mount_hand_root = null
assembly.T_hand_root_grasp_tcp = null
assembly.T_mount_camera_housing = null
assembly.T_housing_optical = null
```

## 检查器和结果

新增 `scripts/inspect_robot_package.py`。它只使用 Python 标准库，支持本地 URDF 或 ZIP 内 URDF，解析 visual/collision mesh、mimic、fixed/non-fixed joints、候选边界、质量字段和每个已解析 mesh 的 SHA256。它拒绝把 Git LFS pointer 当 XML，也拒绝不安全的 ZIP member；它不会推断 actuator ID、action dimension、硬件侧别或物理成功。

输出记录：

- `.local/evidence/m2/omni_left_inspection.json`
- `.local/evidence/m2/omni_right_inspection.json`
- `.local/evidence/m2/airbot_foiegreis_inspection.json`
- `.local/evidence/m2/airbot_discoverse_inspection.json`
- `.local/evidence/m2/discoverse_arm_mesh_tree_20260922.json`
- `.local/evidence/m2/discoverse_arm_mesh_download_20260922.json`
- `.local/evidence/m2/discoverse_arm_mesh_geometry_20260922.json`
- `.local/evidence/m2/discoverse_new_airbot_play_mjcf_20260922.json` and `.local/evidence/m2/discoverse_new_airbot_play_mjx_airbot_play.xml`
- `.local/evidence/m2/discoverse_LICENSE_d67f47c_20260922.txt`
- `.local/evidence/m2/nas_robot_name_probe_20260922.txt`（NAS 文件名探测未发现可直接绑定的 AIRBOT/OmniHand 包）

## 尚未通过的门禁

完成以下输入前，A 只能保持静态证据状态，B 可整理接口但不能宣称 M2 通过：

- 确认两只实物的 O10/O12 型号、修订和解剖左右手；
- 为 AIRBOT Play 锁定与实物匹配的官方/授权 arm package，解析完整 visual/collision 依赖；
- 审阅 DISCOVERSE 同 commit 的 MJCF OBJ 候选与 fixed URDF 的几何、单位、坐标和碰撞等价性；
- 测量并审阅 `flange → mount → hand_root → TCP` 和两路 D405 光学链；
- 核实手、转接件、D405 的质量、质心和惯量，完成 joint zero/sign/effort/damping 与 mimic/actuator 映射；
- 在选定 Isaac Sim/Isaac Lab 环境完成场景打开、mesh/材质依赖、碰撞和 RGB smoke；
- 由 B 通过相机投影与遮挡检查后，才允许 C 接触任务。

本轮没有启动真实机器人、CAN/电机或接触任务，也没有下载整套官方资产库。

## 后续包闭合复核

主会话补取第三方 ROS2 候选 commit `7792960fb60f3118d9827b641dcc441e9ae2d06f` 的 `manipulator/Airbot/airbot_play_description`，完整 visual/collision 引用已解析且无 LFS pointer；证据 `.local/evidence/m2/airbot_ros2_full_package.json`。这取代此前仅有单文件时的缺 mesh 状态，不改变第三方来源或未绑定实物的结论。组合保留关节/惯性/碰撞 XML，静态预览未验证动力学。link6 保留原夹爪座，不可作为裸法兰验收。
