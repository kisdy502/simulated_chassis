# 建图 / 定位 / 导航 使用手册（bridge 统一托管）

本手册适用 dev-lightning 分支：仿真单独启动，其余节点（定位、建图、Nav2、速度转发）
全部由 bridge（`agv_nav_server`）一个入口托管与切换，上位机只面对 `/agv/*` 接口。
SLAM 后端通过 `slam_algorithm` 参数选择（`cartographer` 默认 / `lightning`）。

```
终端一（唯一需要单独启动的）：物理仿真
终端二：bridge —— 托管 定位/建图子进程、Nav2、cmd_vel 转发；组合整机入口还带机械臂运行时
```

## 0. 前提（一次性）

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install            # lightning 默认 COLCON_IGNORE，不参与
source install/setup.bash
```

改动 lightning 源码后用一键脚本重编（自动处理 COLCON_IGNORE 与 Pangolin 路径；
Pangolin 装在 `~/.local`，构建不能用 `--symlink-install`）：

```bash
./scripts/build_lightning.sh            # 默认 -j2 防止 8GB 机器 OOM；MAKE_JOBS=8 可覆盖
```

## 1. 启动

### 终端一：仿真（二选一）

```bash
# 纯底盘（SLAM 验证推荐）
ros2 launch simulated_chassis three_wheel_sim.launch.py
# 需要键盘遥控窗口时
ros2 launch simulated_chassis three_wheel_sim.launch.py start_teleop:=true

# 组合整机（三舵轮 + UR5e，需要图形显示；注意 start_gamepad 默认 false，
# 用手柄遥控须显式开启，否则 joy_node/gamepad_teleop_node 不会启动）
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true start_gamepad:=true
```

### 终端二：bridge

```bash
# 纯底盘入口
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    robot_package:=simulated_chassis \
    slam_algorithm:=lightning \                # 缺省 cartographer
    pbstream_file:=$PWD/maps/my_map            # 首次建图可省略

# 组合整机入口（自动带起 MoveIt / arm_manager / RViz）
ros2 launch simulated_chassis_ur5e bridge.launch.py \
    slam_algorithm:=lightning pbstream_file:=$PWD/maps/my_map rviz:=true
```

`pbstream_file` 两种后端的写法（`maps_dir` 缺省由它推导）：

| 后端 | 写法 | 说明 |
|---|---|---|
| cartographer | `maps/<name>/<name>.pbstream` | 也可以是旧平铺 `maps/<name>.pbstream` |
| lightning | `maps/<name>`（目录）或目录内任意文件 | 地图目录需含 `map.yaml` 与 `global.pcd` |

在 bridge 终端按 Ctrl+C 会同步退出其托管的全部子进程（定位/建图/导航/机械臂/RViz），
等命令提示符回来再重启；仿真终端独立管理。

## 2. 建图（首次，无地图启动）

bridge 启动时**不带** `pbstream_file`，然后：

```bash
# 1. 进入建图（mode=MAPPING，导航任务被拒绝，遥控不受影响）
ros2 service call /agv/start_mapping agv_bridge_v2_interfaces/srv/StartMapping "{}"

# 2. 遥控走图（键盘 / 手柄 / 直发，bridge 常驻转发到底盘）
ros2 run teleop_twist_keyboard teleop_twist_keyboard     # 发布 /cmd_vel

# 3. 覆盖全场后保存（自动停建图 → 用新图拉起定位）
ros2 service call /agv/save_map agv_bridge_v2_interfaces/srv/SaveMap "{map_name: lm_test}"
```

mode 流转：`NAVIGATION → MAPPING →（save_map）→ RELOCALIZING → NAVIGATION`。

- 保存/转换失败时 mode 保持 `MAPPING`，建图不中断，可换名重试。
- 建图子进程异常退出（配置错、话题缺失）时 mode 自动回退 `NAVIGATION`，可重试。

产物落盘 `maps/<map_name>/`，两种后端布局：

| 后端 | 文件 |
|---|---|
| cartographer | `<name>.pbstream` + `<name>.pgm` + `<name>.yaml` |
| lightning | `map.yaml` + `map.pgm`（g2p5 栅格）+ `global.pcd` + 分块 pcd |

## 3. 定位

### 3.1 带图启动

重启 bridge 并传 `pbstream_file`（写法见 §1 表格）。初始位姿规则：

| 后端 | 有记忆位姿（`maps/<name>/last_pose.yaml`） | 无记忆 |
|---|---|---|
| cartographer | 从上次位姿启动轨迹 | 全局搜索（可配） |
| lightning | 注入 init_pose | 建图起点（单位位姿） |

lightning **无全局重定位**：机器人须放置在建图起点附近，或用 §3.2 指定位姿。

### 3.2 运行中重定位 / 切图

```bash
# 在当前/指定地图坐标重启定位（拖拽放点后提交）
ros2 service call /agv/relocalize agv_bridge_v2_interfaces/srv/Relocalize \
  "{map_name: lm_test, x: 1.0, y: -0.5, yaw: 1.57}"

# 切换到另一张图（前后端均要求对应地图工件齐备）
ros2 service call /agv/load_map agv_bridge_v2_interfaces/srv/LoadMap "{map_name: other_map}"

# 列出可加载地图
ros2 service call /agv/list_maps agv_bridge_v2_interfaces/srv/ListMaps "{}"
```

定位子进程崩溃时 bridge 自动按当前地图重启（一次重试）；大 pbstream 加载最长等 300s。

## 4. 导航

定位收敛（mode=NAVIGATION 且 `pose_initialized=true`）后 bridge 自动拉起 Nav2：

```bash
ros2 action send_goal /agv/follow_edge agv_bridge_v2_interfaces/action/FollowEdge \
  "{command_id: t1, node_id: n1, x: 2.0, y: 0.0, theta: 0.0, max_speed: 0.8}"
```

- `MAPPING` / `RELOCALIZING` 模式下导航 goal 直接 REJECT（mode 闸门）。
- 建图中仍可 `/cmd_vel` 遥控（速度转发常驻），支持"边建图边探索"。
- 曲线 / 倒车 / 取消等完整用法见 `src/agv_bridge_v2/README.md`。

## 5. SLAM 后端切换

同一个 bridge 入口，仅 `slam_algorithm` 参数不同，`/agv/*` 接口与 mode 语义完全一致：

```bash
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py robot_package:=simulated_chassis \
    slam_algorithm:=cartographer pbstream_file:=$PWD/maps/carto_map/carto_map.pbstream
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py robot_package:=simulated_chassis \
    slam_algorithm:=lightning   pbstream_file:=$PWD/maps/lm_test
```

行为差异（验证时注意）：

| 项目 | cartographer | lightning |
|---|---|---|
| 全局重定位 | 有 | 无（起点 / relocalize 指定） |
| 建图期 /map、TF | 有（可边建图边看图） | 无（RViz 看不到实时栅格，正常） |
| 定位就绪判据 | /start_trajectory 服务 | /tracked_pose 出现（≤120s） |
| 建图期机器人位姿 | TF map→base_link | /tracked_pose |
| 地图目录布局 | `<name>.pbstream/pgm/yaml` | `map.yaml/map.pgm/global.pcd` |

同一 ROS_DOMAIN_ID 下两套后端不能同时启动，也不要手动另开 cartographer/lightning
launch（与托管子进程双权威冲突）。

## 6. 监控与排障

```bash
watch -n1 "ros2 topic echo /agv/status --once | grep -E 'mode|state|pose_initialized|map_name'"
ros2 topic hz /tracked_pose        # lightning 定位 ~100Hz
ros2 run tf2_tools view_frames     # map→odom→base_footprint→base_link
ros2 run tf2_ros tf2_echo map base_link
```

日志（环境变量 `AGV_PROCESS_LOG_DIR` 可重定向）：

| 日志 | 内容 |
|---|---|
| `/tmp/agv_slam.log` | 托管建图 launch 输出 |
| `/tmp/agv_localization.log` | 托管定位 launch 输出 |
| `/tmp/agv_navigation.log` | 托管 Nav2 输出 |
| `/tmp/agv_lightning_save.log` | `ros2 service call /lightning/save_map` 输出 |

常见问题：

| 现象 | 处理 |
|---|---|
| save_map 后 mode 长时间 MAPPING | 看 `/tmp/agv_slam.log` 与 `/tmp/agv_lightning_save.log`；保存失败可换名重试 |
| lightning 定位 120s 未就绪 | 机器人是否在起点附近？`/points2_1 /points2_2 /imu` 是否有数据？ |
| load_map 报"定位节点不由本节点托管" | bridge 启动须带 `pbstream_file`，手动 launch 的定位无法托管 |
| 建图秒退、mode 回 NAVIGATION | `start_mapping` 参数/配置错，看 `/tmp/agv_slam.log` |
| 编译报 ament_cmake_python "Is a directory" | 混用 symlink/非 symlink 构建所致，删 `build/<pkg>/ament_cmake_python` 重试 |

## 7. 上位机（rosbridge JSON）调用示例

```json
{"op":"call_service","id":"m1","service":"/agv/start_mapping","args":{}}
{"op":"call_service","id":"m2","service":"/agv/save_map","args":{"map_name":"lm_test"}}
{"op":"call_service","id":"r1","service":"/agv/relocalize","args":{"map_name":"lm_test","x":1.0,"y":-0.5,"yaw":1.57}}
```

接口字段全量说明见 `src/agv_bridge_v2/README.md`；后端对比与 lightning 原理见
`docs/lightning_vs_cartographer.md`。
