# lightning 建图/定位 修复记录（2026-11-08）

本文记录 11 月初 lightning 链路几个顽固 bug 的根因、修复方案与源码位置。
涉及提交：`b00c5be` `ce29d50` `13dcd9e` `0c0acdd` `4bfe0c6`（dev-humble 分支）。

## 症状总览

1. 建图初期（不回环）障碍物形状就错乱，3D 点云墙壁扭曲；
2. 修到"不回环时正常"后，一跑回环地图立刻扭曲变形——墙壁拉成线段/三角锥形，
   整图与真实世界差之千里；
3. 回环候选一出现 lightning 进程直接崩溃，建图会话全断；
4. 定位模式启动即退出，上位机无限报"定位未收敛/TF 链路未就绪"；
5. 建图静止时 RViz 里机器人微微抖动。

---

## 一、回环时地图扭曲（误回环）

**症状**：一圈建图 2D 图非常规整，回到原点触发回环后整张图扭曲不成样子。

**根因**（三个因素叠加）：

- `lightning_sim_3d.yaml` 调参前回环从未真正触发过（原 `closest_id_th=50` +
  `kf_dis_th=2.0` 需 100m 才有首个候选），回环→PGO 修正路径从未被验证；
- 回环 NDT 校验的子图**含大面积地面点**：地面平面对齐即可拿高
  TransformationProbability，xy/yaw 约束极弱 → 高分误匹配（上游 #122，
  作者也建议去地面）；
- `max_range=30` 对小仿真世界等于全图都是候选，走廊对面的关键帧也会被配上。

**修复**（`ce29d50`）：

| 改动 | 位置 |
|---|---|
| 回环子图/当前帧均滤除车体系 z<0.1 地面点（拷贝后过滤，不动关键帧原始云） | `src/lightning-lm/src/core/loop_closing/loop_closing.cc` `ComputeForCandidate()` |
| NDT 通过后以 ICP 精化二次校验：fitness>0.2 或偏离 NDT 结果 >0.5m/5° 拒绝 | 同上（上游 #122/#132 建议） |
| `max_range` 30→5：候选只取真邻域 | `src/simulated_chassis/config/lightning_sim_3d.yaml` |
| `max_iteration` 4→6：LIO 收敛更稳，减轻静止位姿噪声 | 同上 |

**验证**：同场景重跑一圈，回环触发后地图仅小幅收缩修正，不再扭曲。

## 二、回环触发即崩溃（Eigen 块越界，SIGABRT）

**症状**：日志出现 `lc candi: 1` 后进程立刻死亡，exit code -6（SIGABRT）。

**根因**：ICP 校验块写错下标——`T_refined.block<3, 3>(0, 3)` 在 4×4 矩阵上
列越界（3+3>4），首个回环候选即触发 Eigen 断言。此 bug 在原提交 `bb8d9ad`
中就存在，也是它当日 7 分钟即被 revert 的真实原因。

**修复**（`4bfe0c6`）：`src/lightning-lm/src/core/loop_closing/loop_closing.cc:279`
改回 `block<3, 3>(0, 0)` 取旋转块。

## 三、初始定位路径解析（定位秒退 + 难排查报错）

**症状**：`lightning 定位进程启动后即退出`，上位机无限
`"map" ... does not exist` / `TF 链路未就绪`。

**根因**：`pbstream_file` 解析逻辑在路径不存在时把**父目录名**当地图名
（`…/maps/map_1108` 解析出 `maps`），并把中间目录传给定位 launch，
`lightning_loc.launch.py:83` 校验目录失败秒退；报错指向中间目录，难排查。

**修复**（`0c0acdd`）：`src/agv_bridge_v2/src/AgvNavServerNode.cpp:91-158`

- 区分"传了地图目录内文件"（父目录含 global.pcd）与"路径无效"（最后一级
  才是意图地图名）两种情况；
- 解析结果无效时按地图名回退到 `maps_dir` 下同名目录（与 launch 侧
  maps_dir 工作区纠正逻辑对齐），WARN 提示；
- 两处均无 global.pcd 时报明确 ERROR，不再静默秒退。

## 四、建图初期形状错乱 / 3D 墙壁扭曲（g2p5 与 TF 链路，b00c5be 前置修复）

11-06 的 `b00c5be` 已解决"不回环也错乱"的底层问题，本次回环修复建立在它之上：

- g2p5 栅格重绘重写：扩边坐标按子栅格对齐、空闲射线过滤无效距离
  （`src/lightning-lm/src/core/g2p5/`，详见 `docs/lightning_upstream_issue_research.md`）；
- map→odom 发布改为**同时间戳重锚定**：在位姿时刻查 odom→base 再组合发布，
  消除"外推位姿 ∘ 旧时刻 odom"双重计数锯齿
  （`src/lightning-lm/src/core/system/map_odom_tf.h`，与 cartographer 链路的
  `tracked_pose_tf_node.cpp` 同款设计）；
- tracked_pose 传播最新关键帧校正，回环修正不再丢
  （`src/lightning-lm/src/core/system/slam.cc` `PublishTrackedPose()`）。

## 五、建图静止时机器人微抖（多实例 TF 冲突）

**症状**：建图静止，RViz 机器人微微抖动；`/tf` 同一时间戳出现多个不同
map→odom 值。

**根因**：launch 失败的历史会话泄漏了 `agv_nav_server` 孤儿进程；再次点
"开始建图"时多个同名服务端同时响应，各自拉起一套 lightning_slam +
dual_lidar_merge，多个 SLAM 实例对同一 topic 发布互相冲突的 TF。

**处置**：`ps` 按 PPID 找孤儿进程清理（父进程为 systemd 即孤儿）。
**防复发建议**（未实施）：给 `agv_rosbridge.launch.py` 关键节点加
`OnProcessExit` 整组退出（`slam.launch.py:109` 对 cartographer 已这样做）。

## 六、launch 脚本执行位缺失

**症状**：`executable 'live_map_http_server.py' not found on the libexec
directory`，launch 中止。

**根因**：git 中该文件模式 100644（无执行位）；工作区为符号链接安装，
launch 的可执行检查失败。

**修复**（`13dcd9e`）：`git update-index --chmod=+x` 纳入执行位；新脚本入库
时注意 `chmod +x` 后再提交。

## 七、运维类问题记录

- **编译 OOM**：虚拟机 7.7G 内存，`-j4` 编译 lightning 重模板文件时 cc1plus
  各占 2~3G 被 OOM killer 杀掉（连 VSCode 一起遭殃）。用
  `MAKE_JOBS=2 bash scripts/build_lightning.sh`；
- **中断的编译会留损坏产物**：构建输出接 `head` 等早退管道会 SIGPIPE 掐死
  编译进程，损坏的 .o 因 mtime "最新"被跳过重编，链出加载即 SIGSEGV 的
  二进制（崩在 dl_init，连 main 都进不去）。处置：删 `build/lightning`
  `install/lightning` 全量重编。构建日志请重定向到文件而非管道截断；
- **增量编译 1s 完成**属正常：单文件粒度，产物在
  `build/lightning/src/liblightning.libs.so`，可执行文件是指向
  `src/lightning-lm/bin/` 的符号链接，动态加载该 .so。
