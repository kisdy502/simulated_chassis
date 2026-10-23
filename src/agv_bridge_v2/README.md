# agv_bridge_v2

当前接口契约（2026-10-23 核对）：[3D 地图 HTTP、URDF/mesh 和地图业务参数/响应](docs/map_model_api.md)。

[腕部相机与按需拍照](docs/camera_api.md)：触发参数/响应、元数据、预览与照片展示能力边界。

AGV 上位机与仿真机器人之间的对接层。**rosbridge 架构**：业务通信暴露标准 ROS 2
接口，由 `rosbridge_server` 对外提供 JSON over WebSocket。独立 C++ 程序
`urdf_mesh_server` 使用 Boost.Beast 提供浏览器需要的 mesh/纹理静态资源。

👉 **上位机对接细节见 [`docs/rosbridge_integration.md`](docs/rosbridge_integration.md)**

机械臂完整契约、关节顺序与连续点动限制见
[`docs/arm_move_integration.md`](docs/arm_move_integration.md)。

复合机器人组合、Lightning 接入和 C++ HTTP 实现见
[`docs/composite_robot_architecture.md`](docs/composite_robot_architecture.md)。

## 对外接口

`agv_nav_server` 业务接口全表（rosbridge 架构「瘦节点」，白名单见 launch）：

| 接口 | 类型 | 说明 |
|---|---|---|
| `/agv/follow_edge` | `agv_bridge_v2_interfaces/action/FollowEdge` | 直线 / 贝塞尔曲线 / 倒车移动 |
| `/agv/set_control` | `agv_bridge_v2_interfaces/srv/SetControl` | start / stop / reset |
| `/agv/status` | `agv_bridge_v2_interfaces/msg/AgvStatus` | 1Hz 状态上报（transient_local；含 mode/map_name/slam_alive/map_age_sec） |
| `/agv/pose` | `geometry_msgs/msg/PoseStamped` | 10Hz 位姿（map 系，与 TF 同源，免解析 /tf） |
| `/agv/start_mapping` | `srv/StartMapping` | 进入建图（mode=MAPPING） |
| `/agv/save_map` | `srv/SaveMap` | 保存建图并自动回到定位 |
| `/agv/load_map` | `srv/LoadMap` | 切换定位地图 |
| `/agv/list_maps` | `srv/ListMaps` | 列出可加载地图 |
| `/agv/get_map` | `srv/GetMap` | 导出某张地图栅格（建图后入库用） |
| `/agv/relocalize` | `srv/Relocalize` | 指定坐标重启定位 |
| `/agv/arm_move` | `srv/ArmMove` | 机械臂绝对关节目标（UR5e 单臂 C++ 适配、双臂 profile；见机械臂专门文档） |
| TF `map → <agv_id>/base_link` | TF | 车体位姿 |

上位机同时通过 rosbridge 直接访问仿真侧的标准话题（默认白名单）：

- 上报（subscribe）：`/odom`、`/tf`、`/tf_static`、`/map`、`/plan`、`/local_plan`、
  `/joint_states`、`/scan_1`、`/scan_2`、`/agv/status`、`/agv/pose`
- 下发（publish）：`/cmd_vel`、`/initialpose`、`/goal_pose`
- 参数读取（params_glob）：`/agv_nav_server/*`、`/robot_state_publisher/*`
  （后者用于 `GET /api/v1/robot/urdf` 读 `robot_description`，F1.3）

**profile 差异**（同一 bridge,launch 入口不同附加能力）：

| 入口 | 附加内容 |
|---|---|
| `agv_bridge_v2 agv_rosbridge.launch.py` | 无附加（纯底盘） |
| `simulated_chassis_ur5e bridge.launch.py` | + C++ `/agv/arm_move` 适配器、MoveIt/arm_manager、URDF mesh 服务 |
| `simulated_dual_arm bridge.launch.py` | + `/agv/arm_move`、合并 `/joint_states`（舵轮+双臂 14+腰 2）、8090 mesh 静态服务 |

**SLAM 后端**：`slam_algorithm:=cartographer|lightning`（默认 cartographer）。
两后端 `/agv/*` 接口与 mode 语义完全一致；建图期契约（/map ≤5s、TF ≥10Hz）由
lightning 补丁 #8 保证。地图根目录固定为工作区 `maps/`，每图独立子目录。

⚠️ **不开放**：`/points2_1`、`/points2_2`（双 3D 雷达点云，JSON 编码数 MB/帧）、
`/imu`（>100Hz）、`/clock`。本仿真栈**没有** `/scan` 和 `/amcl_pose`
（3D 雷达纯定位，非 AMCL）。详见 `docs/rosbridge_integration.md` 第 4.8 节。

## 目录

```text
agv_bridge_v2/                          16 个文件
├── CMakeLists.txt
├── package.xml
├── README.md
├── docs/rosbridge_integration.md       上位机对接指南（协议 + Spring Boot 实操）
├── launch/agv_rosbridge.launch.py      瘦节点 + rosbridge_server + rosapi
├── scripts/agv_client_demo.py          上位机侧参考实现（裸 JSON 协议）
├── include/agv_bridge_v2/
│  ├── AgvNavServerNode.hpp            瘦节点：action/srv/topic/TF
│  ├── NavigationManager.hpp           路径生成 + nav2 调度
│  ├── LocalizationMonitor.hpp         TF 定位收敛监控
│  ├── bean/Edge.hpp                   边的数据结构（纯 POD）
│  ├── bean/MoveToMessage.hpp          一次移动任务的内部表示（纯 POD）
│  └── utils/TransformUtils.hpp        四元数/偏航角/贝塞尔数学
└── src/
    ├── agv_nav_server_main.cpp         入口（MultiThreadedExecutor）
    ├── AgvNavServerNode.cpp
    ├── NavigationManager.cpp
    ├── LocalizationMonitor.cpp
    └── TransformUtils.cpp
```

## 编译

```bash
colcon build --packages-up-to agv_bridge_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
```

先决条件（已写入 `docker/ros2-base/Dockerfile`）：

```bash
sudo apt install ros-humble-rosbridge-suite libboost-system-dev
```

> 本包**不再需要** `libwebsockets-dev` / `libcurl4-openssl-dev` / `nlohmann-json3-dev`。

## 启动

```bash
source install/setup.bash
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py

# 覆盖参数
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    agv_id:=three_wheel_agv \
    port:=9090 \
    use_sim_time:=true \
    back_up_max_heading_error_deg:=20.0
```

## 验证

```bash
ros2 action list | grep agv
ros2 action info /agv/follow_edge
ros2 topic echo /agv/status

# 上位机侧参考实现
pip install websocket-client
python3 src/agv_bridge_v2/scripts/agv_client_demo.py --curve --cancel

# 直接用 ROS 命令行发一条曲线指令
ros2 action send_goal /agv/follow_edge \
    agv_bridge_v2_interfaces/action/FollowEdge \
    "{command_id: 'test1', node_id: 'S008', x: -5.3, y: -0.4, theta: 1.02,
      edge_id: 'E101', edge_type: 'CURVE', max_speed: 0.4, step: 0.05, end_point: true,
      control_points: [{x: -7.1, y: -2.0}, {x: -6.05, y: -0.85}]}" --feedback
```

## 关键行为约定

### 倒车（`back_up=true`）绝不旋转车体

充电桩对接、窄过道通行、贴边作业下原地旋转会剐蹭甚至碰撞。因此：

- 路径点朝向 = 机器人**当前朝向**，位置沿直线插值到目标点
- 配 `setSpeedLimit(-maxSpeed)` 负向限速实现平移后退
- 车尾对目标的朝向偏差 > `back_up_max_heading_error_deg`（默认 20°）时**直接 FAILED**，
  不擅自转向 —— 由调度系统决定先摆正还是改用前进指令

### `end_point=true`

表示这是最后一段，到达后还需按 `theta` 做一次最终旋转（`END_ROTATING`）。

### 单任务

`/agv/follow_edge` 同时只接受一个 goal，已有任务在跑时新 goal 会被 `REJECT`。
停止用 `cancel_action_goal`（`id` 必须与发送时一致）。

### 必须使用多线程执行器

`NavigationManager::cancelNavigation()` 会在回调线程内阻塞等待 nav2 的取消响应 future，
单线程执行器会死锁。`agv_nav_server_main.cpp` 已使用 3 线程的 `MultiThreadedExecutor`。

## 迁移记录：已删除的旧实现

| 删除内容 | 原职责 | 替代者 |
|---|---|---|
| `AgvBridgeNode.{hpp,cpp}`、`main.cpp` | 旧主节点 | `AgvNavServerNode` |
| `websocket_server.{h,hpp,cpp}` | 自研 WS 服务（libwebsockets） | `rosbridge_server` |
| `HttpApiClient.{hpp,cpp}` | HTTP POST 上报（libcurl） | 上位机 subscribe 标准 topic |
| `bean/{BaseMessage,PositionUpdate,LaserScan,StatusUpdate,CommandAck}` | ROS→JSON 手写映射 | `agv_bridge_v2_interfaces` 的 IDL |
| `bean/PathPoint.hpp` | 未被使用的结构体 | — |
| `ImuManager.{hpp,cpp}` | 真机 IMU 串口 → `/imu/data_raw` | ⚠️ 已移除，若仍需此功能请单独建节点 |
| `Edge.cpp`（空文件） | — | — |
| `config/agv_config.yaml`（0 字节） | 旧 HTTP/WS 参数 | launch 参数 |
| `launch/agv_bridge_v2.launch.py` | 旧节点 launch | `launch/agv_rosbridge.launch.py` |

`Edge.hpp` / `MoveToMessage.hpp` 也随之去掉了全部 `to_json` / `from_json` 与
`nlohmann::json` 依赖，`BaseMessage` 继承链一并取消 —— 对外契约完全由 IDL 定义。



curl -X POST http://127.0.0.1:8090/api/v1/scheduler/release-charger 
-H 'Content-Type: application/json' 
-d '{"robot_code": "RB_20260922_00024"}'
