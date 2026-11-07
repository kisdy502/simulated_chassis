# Lightning-LM vs Cartographer 对比测试指南（dev-lightning 分支）

本分支将开源激光 SLAM [lightning-lm](https://github.com/gaoxiang12/lightning-lm)（gaoxiang12，Faster-LIO 前端 + 回环 + 重定位）作为 vendor 包接入仿真工作区，与现有 Cartographer 3D 链路平行运行，用于建图/定位效果与集成复杂度的对比评估。

## 1. 分支改动总览

| 位置 | 内容 |
|---|---|
| `src/lightning-lm/` | 上游 master 的 vendor 副本（去 .git），含接入补丁（见 §3）；`src/core/` 自上游 `1325fed` 恢复并重放补丁 #2/#3/#5 |
| `src/simulated_chassis/src/dual_lidar_merge_node.cpp` | 新增：前后双雷达合流节点（lightning 只支持单雷达话题） |
| `src/simulated_chassis/config/lightning_sim_3d.yaml` | 新增：lightning 仿真配置 |
| `src/simulated_chassis/launch/lightning_slam.launch.py` | 新增：建图链路（对应 slam.launch.py） |
| `src/simulated_chassis/launch/lightning_loc.launch.py` | 新增：纯定位链路（对应 localization.launch.py） |

Cartographer 侧所有文件零改动，两条链路可随时切换对比。

## 2. 编译（Ubuntu 22.04 / ROS2 Humble，arm64 或 x86_64）

```bash
# 在仓库根目录执行；首次在每台机器上安装依赖（需要 apt/sudo）
bash scripts/install_lightning_deps.sh
# 可单独确认 Pangolin 能否被 CMake 发现
bash scripts/build_lightning.sh --check-deps
# 再构建 Lightning（默认两路并行，不使用 --symlink-install）
bash scripts/build_lightning.sh
source install/setup.bash
```

安装脚本使用仓库内固定版本的 `Pangolin-0.9.3.zip`，在当前 CPU 上源码编译，
不下载或复用另一架构的二进制。系统依赖通过 apt 安装；Pangolin 默认安装到
`~/.local`，构建目录位于 Linux 用户缓存，并按 `uname -m` 隔离。
`MAKE_JOBS=2` 可用于两个脚本；内存紧张时降为 1。
Pangolin 是当前源码的强制编译/链接依赖，`with_ui=false` 不会消除该依赖。
更换电脑后需重新安装依赖并构建，不能复制 ARM 的 `build/`、`install/` 或库到 x86。
若曾复制构建产物，应使用当前机器的新构建目录，而不是继续使用另一架构的 CMake 缓存。

构建脚本会在移除 `COLCON_IGNORE` 前检查 Pangolin，退出时恢复忽略标记，
并过滤环境变量中已经不存在的安装前缀。`pcap will be disabled` 是可选 PCAP
功能提示，不阻断当前 ROS 点云输入链路；真正失败原因以 CMake Error 为准。

无 sudo 环境的历史 arm64 手动构建记录（有 sudo 的机器优先使用上方脚本）：

```bash
# 1) libepoxy（Pangolin 0.9.3 依赖，无 root 时 deb 解包到用户目录）
apt download libepoxy-dev libepoxy0 && for d in *.deb; do dpkg -x "$d" ~/.local/stage; done
mkdir -p ~/.local/lib ~/.local/include && cp -rn ~/.local/stage/usr/lib/*_linux-gnu/* ~/.local/lib/ && cp -rn ~/.local/stage/usr/include/* ~/.local/include/

# 2) Pangolin（关 OpenEXR：arm64 上 char 无符号触发 -Werror）
cmake -B build ../Pangolin-0.9.3 -DCMAKE_INSTALL_PREFIX=$HOME/.local -DBUILD_EXAMPLES=OFF \
  -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
  -DCMAKE_INSTALL_RPATH=$HOME/.local/lib -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
make -j && make install

# 3) 工作区编译（lightning 不支持 --symlink-install：嵌套 livox 子包冲突；
#    且默认带 COLCON_IGNORE，直接 --packages-select 会报 unknown package）
cd $WS && ./scripts/build_lightning.sh
source install/setup.bash
```

注意：8GB 内存机器上 lightning 编译需降并行（`make -j2`），-j8 会 OOM。`lightning-lm` 保留 `COLCON_IGNORE`：默认全量 `colcon build` 不依赖 Pangolin 环境（`install/lightning` 一次装好后运行不受影响），改代码后用上面第 3 步命令显式重编（**不能加 `--symlink-install`**，且该模式与其它包的 symlink 构建混用会损坏 `ament_cmake_python` 缓存目录，出现 "existing path cannot be removed: Is a directory" 时删掉对应 `build/<pkg>/ament_cmake_python` 重试）。

## 3. lightning-lm 接入补丁清单（vendor 内修改，均带 `sim-chassis 补丁` 注释）

| # | 文件 | 内容 | 动机 |
|---|---|---|---|
| 1 | `package.xml` | 删除 `scrubber_common`/`agibot_robot` 依赖 | 上游残留的私有包依赖，公开环境无此包 |
| 2 | `src/core/system/loc_system.cc/.h` | `system.pub_tf` 可配置；固定发布 `/tracked_pose`（PoseStamped, map→base_link） | 上游硬发 map→base_link TF，与控制器 odom→base TF 树双权威冲突；改为复用 tracked_pose_tf_node 重锚定 map→odom（与 Cartographer 链路同构） |
| 3 | `src/core/system/slam.cc/.h` | `system.save_dir` 可配置；map.yaml 的 resolution 由硬编码 0.05 改为 g2p5 实际值 | 上游落盘固定 `./data/`（随 cwd 漂移）；resolution 硬编码与实际栅格分辨率不符会导致 map_server 加载错位 |
| 4 | `src/app/run_loc_online.cc` | 支持 `loc.init_pose: [x,y,z,qx,qy,qz,qw]` | 上游初始位姿硬编码单位阵；定位重启/换起点需要 |
| 5 | `src/core/system/{slam,loc_system}.cc` | 传感器订阅 QoS 改为 SensorDataQoS | 上游默认 RELIABLE 订阅与 best_effort 发布端（gz bridge 等）不匹配会静默收不到数据 |
| 6 | `thirdparty/livox_ros_driver/COLCON_IGNORE` | 阻止 colcon 重复发现内嵌包 | 该包由 lightning 的 CMakeLists add_subdirectory 构建 |
| 7 | `cmake/packages.cmake` | SSE 编译选项限定 x86；arm64 跳过 | 上游硬编码 `-msse*` 在 arm64 gcc 不识别，直接编译失败 |
| 8 | `src/core/system/slam.{h,cc}` + `lightning_slam.launch.py` | SLAM 模式实时可视化：发布 `/tracked_pose`（LIO 逐帧位姿）与 `/map`（g2p5 栅格，1Hz 限流，TRANSIENT_LOCAL）；launch 加 tracked_pose_tf_node 重锚定 map→odom | 上游建图期无任何输出话题，RViz 完全看不到进度，问题只能保存后才发现（回环失败导致的鬼影墙等无法在建图中纠正） |
| 9 | `src/core/lio/imu_processing.hpp` | LIO 初始化时世界系对齐重力：R0 = R(mean_acc→+z)，重力恒 (0,0,-G) | 上游 R0=I 且 grav=-mean_acc，初始化窗口（20 批 IMU）内的瞬态（机器人落体回弹/加减速）被固化为世界系恒定倾角——RViz 机器人与地图垂直、g2p5 沿倾斜轴投影产生花纹噪点、定位初始位姿继承倾角 |

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
4. **agv_bridge_v2 已适配（slam_algorithm 参数）**：bridge 通过 `slam_algorithm:=lightning` 切换后端，`/agv/start_mapping`/`/agv/save_map`/`/agv/load_map`/`/agv/relocalize` 全链路走 lightning（详见 §8）；lightning-lm 默认 `COLCON_IGNORE`，不参与 colcon 构建，bridge 通过 ros2 CLI 调 `/lightning/save_map`，无编译期依赖
5. 仿真点云无每点时间戳、无强度标定（intensity 恒定），`lidar_loc.filter_intensity_*` 等滤波在仿真中意义有限

## 8. bridge 集成（slam_algorithm 切换，已实现）

bridge（agv_nav_server）新增 `slam_algorithm` 参数（`cartographer` 默认 | `lightning`），两个后端共用同一套 `/agv/*` 上位机接口与模式机（NAVIGATION/RELOCALIZING/MAPPING）：

| 环节 | cartographer | lightning |
|---|---|---|
| 建图 launch | `slam.launch.py` | `lightning_slam.launch.py`（注入 `save_dir:=<maps_dir>`） |
| 保存 | `/write_state` pbstream + map_saver pgm/yaml | `ros2 service call /lightning/save_map`（CLI 子进程，校验 map.yaml+global.pcd 落盘） |
| 定位 launch | `localization.launch.py`（pbstream_file） | `lightning_loc.launch.py`（map_dir + init_x/y/z/yaw） |
| 初始位姿 | `/start_trajectory` 服务注入 | launch 参数注入（重启式，无全局重定位） |
| 就绪判定 | `/get_trajectory_states` 轮询 | 进程存活 + `/tracked_pose` 新消息（120s 超时） |
| 建图期位姿 | TF map→base_link | `/tracked_pose` 订阅（SLAM 无 TF） |
| 地图校验 | `<name>.pbstream` | `<name>/map.yaml` + `<name>/global.pcd` |

`MapFileManager` 同时识别两种布局（`<name>/<name>.yaml|pgm` 与 lightning 的 `<name>/map.yaml|map.pgm`），`/agv/get_map`、`/agv/list_maps` 对两种后端的地图通用。

```bash
# lightning 全托管（bridge 拉起定位/建图，托管 Nav2）：
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py robot_package:=simulated_chassis \
    slam_algorithm:=lightning pbstream_file:=$PWD/maps/lm_map
# pbstream_file 传 lightning 地图目录（或目录内任意文件）；先建图时可省略。
# 之后 /agv/start_mapping 建图 → 遥控 → /agv/save_map 保存并自动进入定位，
# 与 cartographer 流程完全一致。
```

剩余工作（真机/后续）：全局重定位（上游 TODO，或接受固定起点业务约束）、真机双 Mid-360 的 merger 输入与 time_mode 重评。
