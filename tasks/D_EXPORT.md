# D_EXPORT — 官方 LeRobot v3 写入与回读

后续指定模型：GPT-6 Luna Max（2026-09-23升级；以下历史执行不追溯变更）；2026-09-22 用户已恢复本轮实施，实际启动独立代理，复用并修订旧适配器。官方 SDK synthetic smoke 的具体版本、实际结果和局限见 `docs/LEROBOT_SMOKE.md` 与 `docs/VALIDATION.md`；不将格式小样算作 M4。

## 输入
先读 AGENTS.md、本任务相关 DESIGN 章节和最小所需 manifest；不要整库重复检索。缺少实际输入时提交缺失清单，不能填猜测值。

## 允许修改
packages/sim2data_lerobot/src/sim2data/export/, environments/data/, tests/integration/test_lerobot*

公共 schema、依赖锁、main 与其他代理目录归负责人。

## 工作
先核实安装版本/commit 确实支持 LeRobotDataset v3.0。使用官方 create/add_frame/save_episode/finalize API，构造仅用于格式 smoke test 的小型合成张量/图像回合，明确不能算机器人数据。验证具名 action/state、各相机视频偏移、结束 finalize、时间窗口、stats、故障暂存与单 writer 所有权。只处理已确认 schema；privileged 信息默认放 sidecar。

## 验收
官方 loader 可回读多回合和跨视频分片边界；关闭后重新打开通过；训练 batch 形状正确；缺帧/延迟/NaN/空回合被拒绝；不得只通过 mock writer 就宣称兼容。

## 返回
不超过 500 字摘要：变更、已运行测试、未运行项、关键证据路径、阻塞。完整记录放测试报告。只有真实运行和真实提交后才报告结果或 SHA。
