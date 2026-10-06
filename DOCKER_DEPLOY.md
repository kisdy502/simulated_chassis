# 三舵轮仿真机器人 Docker 部署

当前 Docker 栈按三舵轮机器人 `omni_3wd` 配置，包含三个常驻容器：

| 服务 | 职责 |
|---|---|
| `robot-simulation` | Gazebo Fortress、机器人模型、控制器、雷达和里程计 |
| `nav2-server` | Nav2 导航；不重复启动 Cartographer 定位 |
| `agv-bridge` | rosbridge WebSocket、AGV 业务服务、定位/建图进程管理 |

Cartographer 不再作为第四个常驻容器运行。`agv-bridge` 根据当前模式动态启动
`simulated_chassis/localization.launch.py` 或 `slam.launch.py`，避免同一 ROS domain
出现两个 Cartographer 节点。

## 环境要求

- Windows 11 + Docker Desktop（Linux containers）或 Ubuntu 22.04 Docker
- 建议至少 16 GB 内存；首次基础镜像构建会下载 ROS 2、Nav2、Gazebo 等依赖
- Windows 上从 Git Bash/WSL 执行 `deploy.sh`，或使用下文的 PowerShell 命令

## 配置

默认 `.env` 已适配当前工程：

```dotenv
ROBOT_TYPE=omni_3wd
PBSTREAM_FILE=/ros2_ws/maps/my_map/my_map.pbstream
WEBSOCKET_PORT=9090
START_RVIZ=false
```

地图目录契约为：

```text
maps/<地图名>/<地图名>.pbstream
maps/<地图名>/<地图名>.pgm
maps/<地图名>/<地图名>.yaml
```

## 构建

Git Bash/WSL：

```bash
./deploy.sh build
```

PowerShell：

```powershell
docker build -f docker/ros2-base/Dockerfile -t ros2-base:latest .
docker compose build robot-simulation agv-bridge
docker compose config --quiet
```

业务代码变化通常只需重新执行第二行；只有基础依赖变化时才需要重建
`ros2-base:latest`。

## 启动

Git Bash/WSL：

```bash
./deploy.sh up
```

PowerShell：

```powershell
docker compose up -d --force-recreate robot-simulation nav2-server agv-bridge
docker compose ps
```

上位机连接：`ws://localhost:9090`。

默认是无界面部署，Gazebo 只启动仿真服务端。`START_RVIZ=true` 可在已正确配置
X11 的 Linux 环境中启动 RViz；Windows 日常联调建议保持关闭，在宿主机/WSL
单独运行 RViz。当前 `three_wheel_sim.launch.py` 没有 Gazebo GUI 开关，因此旧的
`GAZEBO_GUI` 环境变量已移除，避免造成“设为 true 就会弹窗”的误解。

## 建图、保存地图和重定位

进入建图模式：

```bash
./deploy.sh up-slam
```

该命令先启动三个常驻容器，再调用 `/agv/start_mapping`。建图过程中可由上位机向
`/cmd_vel` 发布速度控制机器人探索。

保存地图并自动切回定位：

```bash
./deploy.sh save-map map_001
```

上位机通过 rosbridge 调用时：

```json
{"op":"call_service","service":"/agv/start_mapping","args":{}}
{"op":"call_service","service":"/agv/save_map","args":{"map_name":"map_001"}}
{"op":"call_service","service":"/agv/relocalize","args":{"map_name":"my_map","x":0.0,"y":0.0,"yaw":0.0}}
```

地图切换和重定位还可使用 `/agv/list_maps`、`/agv/load_map`。实际服务字段应以
`src/agv_bridge_v2_interfaces/srv/` 中的定义为准。

## 验证与排障

```bash
docker compose ps
docker compose logs --tail 200 robot-simulation
docker compose logs --tail 200 nav2-server
docker compose logs --tail 200 agv-bridge

docker exec agv_bridge bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 service list | grep /agv"

docker exec robot_simulation bash -lc \
  "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 topic list"
```

关键话题应包括 `/odom`、`/scan_1`、`/scan_2`、`/joint_states`；关键服务应包括
`/agv/start_mapping`、`/agv/save_map`、`/agv/load_map`、`/agv/relocalize`。

常用维护命令：

```bash
./deploy.sh status
./deploy.sh logs agv-bridge
./deploy.sh restart
./deploy.sh down
```

如果 9090 被 WSL 中旧的 rosbridge 占用，先停止旧进程或在 `.env` 修改
`WEBSOCKET_PORT`，并让上位机使用相同端口。
