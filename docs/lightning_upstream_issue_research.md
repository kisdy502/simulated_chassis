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

## 中央墙扭曲追查（2026-11-07）

现象：对比 gazebo 世界，中央走廊墙（inner_h1a/h1b + inner_h2）在 3D 图中扭曲，
2D 投影坍缩/断裂，四周外墙有重影。对 `map1105/map.pgm` 的定量复核：外框内容
56m×47.4m 与世界尺寸一致（刚性好，无整体缩放）；inner_h2 断成两截、各外偏 3~4m、
中段缺失；inner_v2 在预期位置缺失、碎片位移 2~4m；空地出现幽灵线；右外墙双线。
11/06 live 诊断实测 `map←odom` z 偏移 −0.43m（LIO z 漂移）。

### 上游对应结论（gaoxiang12/lightning-lm）

- **#137**（关闭）`nav_state.h` `oplus()` 的 `vel_ +=` 被注释是“手误”，答复“应当解注释”。
  **#142**（开放）报告者在与本 vendor 相同的 commit `1325fed` 上验证解注释可消除纯平移
  位姿过冲；但其同主题后续评论（07-10）警告：抖动较大的平台上该状态更新会导致前端不稳。
- **#125**（开放）`predict_cov_inflation_=1.01` 硬写入在车辆振荡后静止/减速时使 P 异常发散（z 漂）。
- **#144**（开放）ESKF 观测退化特殊处理（HTH 特征值掩码 + 协方差膨胀）致 `dx_current` 异常；
  作者答复：“这部分确实效果不好，**可以关掉**。退化应该引入轮速计，单 IMU+lidar 缺速度观测。”
- **#122**（关闭）NDT 高 TransformationProbability 的误回环：作者建议“把地面部分去了，
  roi height_min 设高”；提问用户补充“**加上 icp 做二次校验，还挺有效**”。
- **#5**（开放）走廊退化实测：拐角重影、“直接飞起来”；建议 IMU 噪声调大 + 在线标定。
- **#29**（关闭）z 漂移：作者称 2025-11 已修，但有用户 with_height=true 仍漂；曾提议“固定高度选项”。
- **#156**（开放）长距离建图 banana effect（两端高中间低）：无人回复，无官方解。

### 本地试验记录

1. **#137+#125 组合（已回退）**：恢复速度递推 + 膨胀 1.0。实测重影反而加重
   （本场景为双雷达合流快照点云、ESKF 仅 6 维位姿观测、无加计零偏状态，
   与 #142 后续评论的警告一致）。已恢复上游原状。
2. **#144（生效中）**：`eskf.hpp` `degeneracy_threshold_ratio_` 1e-3→0、
   `degeneracy_cov_inflation_` 1.02→1.0，整体关闭退化特殊处理。
3. **#122（生效中）**：`loop_closing.cc` 回环子图/当前帧点云拷贝后 `RemoveGround(z>0.1)`
   去地面；NDT 后追加 ICP 精化+校验（不收敛/fitness>0.2/偏移>0.5m/转角>5° 拒绝）。

### 验收要点（新一轮建图，新地图名）

- 中央走廊墙连续（inner_h2 应为单条 ~0.72m 厚、26m 长）、无外偏断截；
- 回环日志关注 `loop rejected by icp check` 条目（说明 ICP 校验在拦截误回环）；
- 右外墙双线、`map←odom` z 偏移（此前 −0.43m）对比；
- 若走廊段仍有残余扭曲，下一步按 #144 作者意见评估引入轮速计观测（仿真已有 `/odom`，
  ESKF 已有 `ObsType::WHEEL_SPEED` 枚举但未接线）。
