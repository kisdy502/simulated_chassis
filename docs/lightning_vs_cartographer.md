# Lightning-LM vs Cartographer 对比测试指南（dev-lightning 分支）

本分支将开源激光 SLAM [lightning-lm](https://github.com/gaoxiang12/lightning-lm)（gaoxiang12，Faster-LIO 前端 + 回环 + 重定位）作为 vendor 包接入仿真工作区，与现有 Cartographer 3D 链路平行运行，用于建图/定位效果与集成复杂度的对比评估。

## 1. 分支改动总览

| 位置 | 内容 |
|---|---|
| `src/lightning-lm/` | 上游 master 的 vendor 副本（去 .git），含 5 处接入补丁（见 §3） |
| `src/simulated_chassis/src/dual_lidar_merge_node.cpp` | 新增：前后双雷达合流节点（lightning 只支持单雷达话题） |
| `src/simulated_chassis/config/lightning_sim_3d.yaml` | 新增：lightning 仿真配置 |
| `src/simulated_chassis/launch/lightning_slam.launch.py` | 新增：建图链路（对应 slam.launch.py） |
| `src/simulated_chassis/launch/lightning_loc.launch.py` | 新增：纯定位链路（对应 localization.launch.py） |

Cartographer 侧所有文件零改动，两条链路可随时切换对比。

## 2. 编译（Ubuntu 22.04 / ROS2 Humble，或项目 docker）

```bash
# lightning 额外系统依赖（一次性）
sudo apt install libglog-dev libgflags-dev libyaml-cpp-dev
# Pangolin-0.9.3（编译期必需；运行无 UI 也需要）
cd /tmp && unzip $WS/src/lightning-lm/thirdparty/Pangolin-0.9.3.zip
cd Pangolin && mkdir build && cd build && cmake .. && make -j && sudo make install

# 工作区编译（lightning 内嵌 livox 驱动已加 COLCON_IGNORE，不会重复构建）
cd $WS
colcon build --packages-select lightning simulated_chassis
source install/setup.bash
```

## 3. lightning-lm 接入补丁清单（vendor 内修改，均带 `sim-chassis 补丁` 注释）

| # | 文件 | 内容 | 动机 |
|---|---|---|---|
| 1 | `package.xml` | 删除 `scrubber_common`/`agibot_robot` 依赖 | 上游残留的私有包依赖，公开环境无此包 |
| 2 | `src/core/system/loc_system.cc/.h` | `system.pub_tf` 可配置；固定发布 `/tracked_pose`（PoseStamped, map→base_link） | 上游硬发 map→base_link TF，与控制器 odom→base TF 树双权威冲突；改为复用 tracked_pose_tf_node 重锚定 map→odom（与 Cartographer 链路同构） |
| 3 | `src/core/system/slam.cc/.h` | `system.save_dir` 可配置；map.yaml 的 resolution 由硬编码 0.05 改为 g2p5 实际值 | 上游落盘固定 `./data/`（随 cwd 漂移）；resolution 硬编码与实际栅格分辨率不符会导致 map_server 加载错位 |
| 4 | `src/app/run_loc_online.cc` | 支持 `loc.init_pose: [x,y,z,qx,qy,qz,qw]` | 上游初始位姿硬编码单位阵；定位重启/换起点需要 |
| 5 | `src/core/system/{slam,loc_system}.cc` | 传感器订阅 QoS 改为 SensorDataQoS | 上游默认 RELIABLE 订阅与 best_effort 发布端（gz bridge 等）不匹配会静默收不到数据 |
| 6 | `thirdparty/livox_ros_driver/COLCON_IGNORE` | 阻止 colcon 重复发现内嵌包 | 该包由 lightning 的 CMakeLists add_subdirectory 构建 |

## 4. 建图对比测试

### 4.1 Lightning 链路

```bash
# 1. 仿真机器人（terminal A）
ros2 launch simulated_chassis three_wheel_sim.launch.py

# 2. lightning 建图（terminal B；本机带显示器可加 with_ui:=true）
ros2 launch simulated_chassis lightning_slam.launch.py

# 3. 手柄/键盘遥控行驶覆盖全场（与 carto 建图走相同路线，便于对比）

# 4. 保存地图（地图落盘 $PWD/maps/lm_map/）
ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: lm_map}"
```

### 4.2 Cartographer 链路（对照组）

```bash
ros2 launch simulated_chassis three_wheel_sim.launch.py
ros2 launch simulated_chassis slam.launch.py            # 相同路线行驶
# 结束后 occupancy_grid/carto 保图流程照旧（pbstream 三件套）
```

### 4.3 对比观察点

| 项目 | Cartographer | Lightning-LM |
|---|---|---|
| 建图可视化 | RViz 实时 /map + 点云 | Pangolin UI（with_ui）或事后 `pcl_viewer maps/lm_map/global.pcd` |
| 2D 栅格 | occupancy_grid_node 实时 /map | g2p5 离线产物 map.pgm（建图时 with_g2p5） |
| 回环 | pose_graph 约束 | loop_closing + miao 优化 |
| 资源 | `top`/`htop` 记录两进程 CPU%、RSS | 同左 |

## 5. 定位对比测试

```bash
# 1. 仿真机器人（放在建图起点，两链路要求一致）
ros2 launch simulated_chassis three_wheel_sim.launch.py

# 2. Nav2 全栈（不开 carto 定位）
ros2 launch simulated_chassis navigation.launch.py include_localization:=false

# 3a. Lightning 定位（terminal C）
ros2 launch simulated_chassis lightning_loc.launch.py map_dir:=$PWD/maps/lm_map

# 3b. 或 Cartographer 对照
ros2 launch simulated_chassis localization.launch.py pbstream_file:=...
```

验证项：

- `ros2 run tf2_tools view_frames`：TF 树应为 `map → odom → base_footprint → base_link`（两链路结构一致，`map→odom` 均由 tracked_pose_tf_node 发布）
- `ros2 topic hz /tracked_pose`（lightning 约 100Hz，IMU 外推）
- RViz：FixedFrame=map 下机器人轨迹平滑度、/map 显示
- Nav2 下发目标点，观察全程定位稳定性（含原地旋转、倒车）

## 6. 定量评估（真值对账）

仿真优势：Gazebo 真值可随时获取（`ign topic -e -t /model/three_wheel_agv/odometry`，注意仅用于离线评估，不桥入 ROS）。

- 录制对照 bag：`/tracked_pose`、`/odom`、`/tf`、真值 odometry（rosbag 录 gz 侧需临时桥一路，或用 `ign topic` 落盘）
- 指标：同时刻 `(map→base)_slam` vs 真值位姿的平移/角度误差曲线（可复用仓库根目录 `plot_odom.py` 的思路）
- 场景：静止 60s 漂移、直线 10m、8 字/原地旋转、重访区域（回环）、快速变速

## 7. 已知限制（对比结论需注明）

1. **无全局重定位**：lightning 必须已知初始位姿（init_pose 或放回建图起点），绑架/丢定位无法自动恢复；Cartographer 有全局约束搜索
2. **双雷达合流为自研节点**：合并帧 8640 点@15Hz，时间对齐策略 time_mode=zero（快照点云无帧内畸变）；真机接入时需重评（可切 linear）
3. **建图过程无 /map、无 TF**：Nav2 无法在建图同时导航（Cartographer 可以边建图边导航）
4. **agv_bridge_v2 未适配**：本分支定位/建图仅支持手动 launch，`/agv/load_map`、`/agv/start_mapping`、save_map 托管仍走 Cartographer；若对比胜出，桥接层适配工作量见 §8
5. 仿真点云无每点时间戳、无强度标定（intensity 恒定），`lidar_loc.filter_intensity_*` 等滤波在仿真中意义有限

## 8. 若决定切换：剩余工作预估

| 模块 | 工作量 | 说明 |
|---|---|---|
| agv_nav_server 适配 | 大 | save_map 改调 /lightning/save_map；load_map 重启改为 map_dir 参数；/start_trajectory 初始位姿注入改为 init_pose 配置；就绪检测从 /get_trajectory_states 改为 TF 探活 |
| MapFileManager | 中 | pbstream 检查改为地图目录（pcd 分块）检查；pgm/yaml 逻辑复用 |
| 全局重定位 | 大（可选） | 上游 TODO；或接受"固定起点/人工 init_pose"业务约束 |
| 真机双 Mid-360 | 中 | merger 输入换 livox CustomMsg 或保留 ROS 点云；time_mode 重评 |
