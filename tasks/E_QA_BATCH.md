# E_QA_BATCH — 随机化质量门禁与批处理

指定模型：GPT-5.6 Luna Max；2026-09-22 第二波仅启动 plan-only 计划器，后按用户要求暂停，内容留在独立 worktree 未合入。M3/M4 前禁止扩为批量采集。

## 输入
先读 AGENTS.md、本任务相关 DESIGN 章节和最小所需 manifest；不要整库重复检索。缺少实际输入时提交缺失清单，不能填猜测值。

## 允许修改
sim2data/randomization/, sim2data/qa/, scripts/batch_*, tests/test_random*

公共 schema、依赖锁、main 与其他代理目录归负责人。

## 工作
优先建立测试，不在 M3/M4 之前启动大规模数采。使用不依赖 worker ID 的 component seed，记录实际域随机化参数；尺寸/碰撞/惯性和抓取候选保持一致。先一环境后逐级扩并行；有界队列与每 writer 独立输出；完成标记、崩溃去重、成功数据筛选和 train/val 按 episode/object identity 分组。

## 验收
报告成功 episodes/hour、分层失败率、显存/内存、物理/渲染/编码/IO时间与GB/episode；无相邻环境串景/串影、无跨 episode 串帧；恢复测试无重复ID；不用随机 seed 声称GPU位级确定。

## 返回
不超过 500 字摘要：变更、已运行测试、未运行项、关键证据路径、阻塞。完整记录放测试报告。只有真实运行和真实提交后才报告结果或 SHA。
