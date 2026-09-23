# A_ASSETS — 官方资产与机器人接口审计

后续指定模型：GPT-6 Luna Max（2026-09-23升级；以下历史执行不追溯变更）；2026-09-22 用户已恢复本轮实施，实际启动独立代理并审阅旧 worktree。候选/静态审计 manifest 已交付，来源绑定和物理未决项以 `configs/asset_manifest.json`、`docs/ASSET_AUDIT.md` 为准；不宣称 A 的生产验收通过。

## 输入
先读 AGENTS.md、本任务相关 DESIGN 章节和最小所需 manifest；不要整库重复检索。缺少实际输入时提交缺失清单，不能填猜测值。

## 允许修改
packages/sim2data_core/src/sim2data/assets/, configs/asset_manifest.*, tests/test_assets*

公共 schema、依赖锁、main 与其他代理目录归负责人。

## 工作
读取只读盘点报告、实际已用 AIRBOT/OmniHand 模型和 SDK 配置；优先定位本地已有资产。区分普通/Pro/硬件修订与左右手，列出主动电机、物理关节、耦合映射、控制单位和限制。定位 card_box 并用真实 USD 打开验证单位、upAxis、尺度、碰撞、质量和材质。展开 USD/纹理/MDL/payload 依赖，列出缺失与远程依赖，不默认下载。只对最终选用文件做哈希与来源/许可记录。

## 验收
一个可审阅 manifest；未解决项明确报错；全新路径可解析；LFS pointer 不能被误当真实资产；不得猜测 DOF 或将所有 URDF 非 fixed joints 当独立驱动。

## 返回
不超过 500 字摘要：变更、已运行测试、未运行项、关键证据路径、阻塞。完整记录放测试报告。只有真实运行和真实提交后才报告结果或 SHA。
