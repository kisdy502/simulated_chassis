# Lightning-LM 上游栅格重影调查（2026-11-06）

本地反馈是保存的 `global.pcd` 主体轮廓基本完整，`map.pgm` 出现擦掉的轮廓和射线状痕迹。上游 #154 报告了相近的三维点云与二维栅格差异。上游 #126 还给出了可把地图扩边与空闲射线分开排查的对照实验。

## 上游报告

- **#154，回环后栅格重绘异常，仍开放。** 报告者在 Jetson NX 上开启回环，称三维点云已经对齐，但用 G2P5 的 `ToRos` 得到的栅格在回环区域异常。报告者已经试过调整 `visit_cnt > 1` 和应用 PR #113 所述修正。评论认为像是 Resize 问题，并建议试用 #126 的改法。讨论没有后续验证结果。 [Issue #154](https://github.com/gaoxiang12/lightning-lm/issues/154)
- **#126，地图扩边引入栅格重影，已关闭。** 报告者屏蔽 `SetWhitePoints`，只画命中点，对比多次 `ResizeMap` 与预先设置足够大的边界，重影出现在多次扩边的一组。报告者指出边界对齐应使用整个 `SubGrid` 的边长。关闭状态说明该报告已被标记完成，页面没有关联的合并记录。 [Issue #126](https://github.com/gaoxiang12/lightning-lm/issues/126)
- **PR #113，修正 `G2P5Map::Resize`，仍开放。** 提案改变旧块复制范围的取整，并把保存的原点对齐到子栅格边界，再由原点和分配的块数计算最大边界。该提案与 #126 都涉及扩边坐标，改动落点不同。 [PR #113 的差异](https://github.com/gaoxiang12/lightning-lm/pull/113/files)
- **#153，Jetson NX 误回环，仍开放。** 同一报告者称，同一个 bag、同一份配置在 x86 未出现该误回环，在 Jetson NX 上却出现。报告没有回复或已确认的根因，也没有证据将此误回环与 #154 的栅格异常归为同一故障。 [Issue #153](https://github.com/gaoxiang12/lightning-lm/issues/153)

## 本地代码与现象

- **扩边时已有占据数据可能发生平移。** 本地 `ResizeMap` 用 `GetGridResolution()` 对齐边界，该接口返回单格分辨率 0.05 m。`G2P5Map::Resize` 按 16×16 子栅格复制，每块边长 0.8 m，并将输入的最小边界直接保存为新原点。例如 X 原点从 0 扩展到 −0.05 m，复制偏移取 `round(−0.05 / 0.8) = 0`，旧块索引不动，其单元对应的世界坐标却整体向负 X 移动 0.05 m。后续命中按新原点入格，可能和已存轮廓错开。这与 #126 描述的扩边机制一致，实际图中受它影响的范围仍需回放验证。 [扩边调用](../src/lightning-lm/src/core/g2p5/g2p5.cc#L218) · [分辨率定义](../src/lightning-lm/src/core/g2p5/g2p5_map.h#L97) · [子栅格复制](../src/lightning-lm/src/core/g2p5/g2p5_map.cc#L54) · [仿真配置](../src/simulated_chassis/config/lightning_sim_3d.yaml#L62)
- **无效距离产生的空闲射线是另一条路径。** 旧 `SetWhitePoints` 在无效距离分支里，对 `r < 0.1` 仍调用 `SetMissPoint`，可把负半径哨兵当作空闲空间观测。当前工作区的改动让 `r <= 0` 或超量程方向直接跳过。此路径与轮廓被擦掉、出现射线的现象相符，修复效果仍需重新绘图确认。#126 在关闭空闲射线后仍报告扩边重影，因此应单独验证扩边。 [空闲射线代码](../src/lightning-lm/src/core/g2p5/g2p5.cc#L398) · [Issue #126](https://github.com/gaoxiang12/lightning-lm/issues/126)
- **两种保存产物经过不同路径。** `global.pcd` 由 LIO 的全局点云生成，`map.pgm` 来自 G2P5 最新栅格的 `ToROS()`。若同一轮保存中点云轮廓稳定而栅格异常，优先用同一组关键帧和优化位姿分别对照固定边界与动态扩边，再分别启用与屏蔽空闲射线。若点云或关键帧轨迹也在回环后跳变，则沿误回环方向检查。 [保存实现](../src/lightning-lm/src/core/system/slam.cc#L191) · [本地回环排查](lightning_mapping_diagnostics.md)
