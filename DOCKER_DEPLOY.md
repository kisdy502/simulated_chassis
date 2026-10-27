# 仿真机器人 Docker 部署

本文对应当前仓库的 `docker-compose.yml`、`deploy.sh` 和 Docker 启动脚本。
默认部署是三舵轮底盘、双 3D 雷达、UR5e 六轴机械臂和腕部仿真相机，
使用 ROS 2 Humble、Gazebo Fortress、ros2_control、MoveIt 和 Nav2。
SLAM 支持 Lightning-LM 与 Cartographer。上位机、Web 前端及其数据库等由各自项目部署。

## 1. 容器与进程架构

只有两个 Compose 服务。`ros2-base:latest` 是构建基础镜像，不是运行服务。

```text
上位机 / Web 前端
  ├─ WebSocket :9090 → :9090 ─ rosbridge_server / rosapi
  ├─ HTTP :8089 → :8089 ────── map_3d_server（已保存地图的三维点云）
  └─ HTTP :8090 → :8090 ────── urdf_mesh_server（模型 mesh / 纹理资源）
                                      │
                         agv-bridge 容器 172.28.0.11
                         ├─ agv_nav_server：地图、任务与状态接口
                         ├─ 托管 Lightning 或 Cartographer、定位、Nav2
                         ├─ map_3d_server + 两种点云导出器
                         ├─ urdf_mesh_server
                         └─ 就绪检查 → MoveIt / arm_manager / arm_move_adapter
                                      ↕ ROS 2 DDS
                         robot-simulation 容器 172.28.0.10
                         ├─ Gazebo：一个世界、一个底盘 + UR5e 整机模型
                         ├─ 一个 controller_manager：底盘控制器 + 机械臂 JTC
                         ├─ joint_state_broadcaster / robot_state_publisher
                         ├─ 双雷达、IMU、里程计、/clock
                         └─ 相机渲染 → ros_gz_bridge → wrist_camera_node
```

| Compose 服务 | 镜像 | 启动入口 | 职责 |
|---|---|---|---|
| `robot-simulation` | `simulated-robot:wheel-3d` | `/launch_robot.sh` | 整机物理仿真、控制器、传感器、相机和拍照 |
| `agv-bridge` | `agv-bridge:wheel-3d` | `/launch_bridge.sh` | 对外通信、地图与导航生命周期、三维地图、模型资源、机械臂业务运行时 |

默认 `ROBOT_PROFILE=chassis_3d_ur5e` 分别启动组合包的
`simulation.launch.py` 和 `bridge.launch.py`。机械臂运行时先检查整机模型与控制器，
通过后才启动。重启 bridge 不会重新生成机器人或将机械臂恢复折叠姿态。

`ROBOT_PROFILE=chassis_3d` 使用纯底盘入口，不启动 UR5e、腕部相机、机械臂业务节点
或 8090 模型资源服务；8089 地图服务仍由通用 bridge 启动。
当前 Docker 脚本只接受这两个 profile 和 `ROBOT_TYPE=omni_3wd`。
仓库中的双臂、其他机械臂演示包没有因此自动加入 Docker 部署。

定位、建图和 Nav2 由 bridge 统一托管，不再额外启动独立导航或 SLAM 容器。
`depends_on: service_started` 只保证容器启动顺序，不代表所有 ROS 节点已就绪。

## 2. 对外端口与上位机接入

以下端口是默认值；`HOST` 表示部署机器的地址。

| 地址 | 提供方 | 用途 | 宿主端口配置 |
|---|---|---|---|
| `ws://HOST:9090` | `rosbridge_server` | ROS 话题订阅/发布、服务和 Action、rosapi 参数读取 | `WEBSOCKET_PORT`，容器固定 9090 |
| `http://HOST:8089/maps/<name>/status` | `map_3d_server` | 查询三维地图导出状态 | `MAP_3D_PORT`，容器固定 8089 |
| `http://HOST:8089/maps/<name>/cloud.ply` | `map_3d_server` | 下载三维地图点云 | 同上 |
| `http://HOST:8090/<pkg>/<relative-path>` | `urdf_mesh_server` | 下载 ROS 包内 mesh、纹理等资源 | `URDF_MESH_PORT`，容器固定 8090 |

### Windows、WSL 与 Docker 的访问地址

端口是否冲突取决于监听地址和网络命名空间，而非是否运行相同程序。
WSL 的 localhost 转发/镜像网络与 Docker Desktop 发布端口都可能使用 Windows
宿主入口；两台机器人若都通过 `localhost:9090` 访问，这个地址没有机器人选择能力。
因此本部署约定：WSL 与 Docker 仿真交替运行，先完整停止一套的仿真和 bridge，
再启动另一套。两种方式均使用标准端口 9090、8089、8090，不增加端口偏移。
如果以后需要同机同时运行，再为其中一套配置不同宿主机端口。
Docker 内部端口、ROS launch 默认端口和 Dockerfile EXPOSE 均保持原值。

| 服务 | Windows 访问 WSL 机器人 | Windows/浏览器访问 Docker 机器人 | 同 Docker 网络的上位机访问 Docker 机器人 |
|---|---|---|---|
| WebSocket / ROS API | `ws://localhost:9090` | `ws://localhost:9090` | `ws://agv-bridge:9090` |
| 3D 地图 HTTP | `http://localhost:8089` | `http://localhost:8089` | `http://agv-bridge:8089` |
| mesh/纹理 HTTP | `http://localhost:8090` | `http://localhost:8090` | `http://agv-bridge:8090` |

Windows 访问 WSL 通常可用 localhost；用宿主机局域网 IP 访问 WSL 则还取决于
NAT/镜像网络模式、监听地址、防火墙及端口转发，不能当作始终等同。
远程电脑浏览器中的 localhost 指向远程电脑自身，应使用部署机器的可达 IP/域名。
详见 [WSL 网络说明](https://learn.microsoft.com/windows/wsl/networking) 和
[Docker 端口发布说明](https://docs.docker.com/engine/network/port-publishing/)。

Docker 上位机与机器人必须加入同一个 Docker 网络才可用服务名 `agv-bridge`；
本项目默认网络名为 `simulated_chassis_agv_network`（项目名改变时以
`docker network ls` 为准）。其他 Compose 项目可用 external network 加入该网络。
上位机容器内 localhost 指向上位机容器自身，不能用它选择 WSL 或另一个机器人容器。

**浏览器访问地址与后端访问地址分别配置：**

- Docker 后端连接机器人：`ws://agv-bridge:9090`。
- Docker 后端代理地图时：`MAP3D_BASE_URL=http://agv-bridge:8089`。
  浏览器访问该后端的 `/api/v1/map-3d/...`，不直接访问机器人；此时无需让浏览器使用 8089。
- 浏览器直连 Docker 地图服务时：用 `http://HOST:8089`。
- 浏览器直接加载 Docker mesh 时：`VITE_URDF_MESH_BASE=http://HOST:8090`，
  不能写只有 Docker 内部可解析的 `agv-bridge:8090`。当前约定下同一宿主地址
  指向当时运行的那套机器人，不用于同时选择两台机器人。
  若 mesh 也由后端反向代理，浏览器改用该代理的外部地址，代理内部仍访问 `agv-bridge:8090`。
- URDF 文本由后端经所选机器人的 WebSocket/rosapi 读取；仅改 mesh URL 不会切换
  URDF 或 `/joint_states` 的来源。每套上位机的 WebSocket、地图来源和 mesh 来源必须一致。
- `VITE_*` 通常为前端构建配置，更改后需按上位机项目的方式重建前端；
  本仓库不包含上位机/前端 Compose，不能自动替它们修改连接配置或发布端口。

**ROS domain 保留独立配置：** WSL 节点使用 `ROS_DOMAIN_ID=0`，Docker 的两个容器使用
`ROS_DOMAIN_ID=1`（已设为 Compose 默认）。启动 WSL 时需给所有相关进程使用同一 domain，
包括仿真和 bridge。AGV_ID 也应不同，例如 WSL `AGV001`、Docker `AGV002`。
仅换 TCP 端口或 AGV_ID 不能隔离 ROS `/clock`、TF、关节状态与控制话题。

当前部署额外端口清单：机械臂、相机/拍照、电池、二维地图、rosapi 均复用 ROS DDS
及 9090 rosbridge，没有各自独立的 HTTP/TCP/WebSocket 监听端口；
Nav2、MoveIt、Gazebo 和 DDS 的内部通信不发布到宿主机。
当前对外 TCP 服务共三项，已全部列入上表。若未来新增外部监听服务，新增对应的
宿主端口映射并检查是否占用；无需修改 ROS 内部动态 UDP 端口。

配置修改后可在安排停机时执行 `bash deploy.sh restart` 重建两容器；
仅改此端口映射不需要重编译镜像，`docker compose restart` 则不会更新端口绑定。
检查 `docker compose ps` 应显示 `9090->9090`、`8089->8089`、`8090->8090`。

### URDF 文本与模型资源是两条链路

整机 URDF 文本来自 `robot_state_publisher` 的 `robot_description` 参数，
上位机通过 rosbridge/rosapi 读取。上位机项目中的 `/api/v1/robot/urdf`
是其自己的接口，不是机器人 8090 服务提供的接口。

8090 的 C++ `urdf_mesh_server` 通过 ament 索引定位已安装的 ROS 包资源。例如：

```text
package://ur_description/meshes/ur5e/visual/base.dae
  → http://HOST:8090/ur_description/meshes/ur5e/visual/base.dae
```

前端使用 `VITE_URDF_MESH_BASE` 时，应将其配置为浏览器可访问的
`http://HOST:8090`（宿主端口改变时同步修改）。服务支持 GET/HEAD/OPTIONS 和 CORS，
不提供目录列表。能连接 9090 不代表模型资源的 HTTP 地址也配置正确。

### 三维地图服务

8089 运行 C++ `map_3d_server`，后台调用 `pbstream_to_ply` 或 `pcd_to_ply`：

- Cartographer：从保存的 `.pbstream` 导出三维占据点云。
- Lightning：从地图目录的 `global.pcd` 降采样导出 PLY。
- bridge 根据 `SLAM_ALGORITHM` 指定来源格式，避免同名地图混用后端。

状态为 `generating`、`ready` 或 `failed`；PLY 未就绪返回 409，地图不存在返回 404。
请在地图保存完成后查询。PLY 使用 ROS `map` 坐标，单位米，Z 向上。
这是已保存地图的点云浏览功能；二维 PGM/YAML 与 Nav2 仍继续使用。

上位机若使用现有 Java 代理，接口为 `/api/v1/map-3d/<name>/status` 和
`/api/v1/map-3d/<name>/cloud.ply`，其机器人服务地址可通过上位机的
`MAP3D_BASE_URL` 配置；该变量不属于本仓库 Compose。

### 通信方向

上位机连接机器人暴露的 WebSocket/HTTP 服务。当前 Compose 不使用旧的
`SPRINGBOOT_HOST`、`SPRINGBOOT_PORT` 或 `host.docker.internal` 回连配置。
外部应用通过宿主机发布端口访问即可，无需为了对接加入 ROS DDS 网络。
WebSocket 的话题、服务和 Action 受 launch 白名单限制，详见
[rosbridge 对接文档](docs/rosbridge_integration.md)。

## 3. 首次配置与构建

在 Linux 或 WSL 的仓库根目录执行。宿主机需要 Docker、Compose v2 和 Bash；
宿主机无需预装 ROS。先确认 `docker info` 和 `docker compose version` 可用。
首次构建需要访问镜像源和 apt 软件源。

```bash
# 仅首次创建；已有 .env 时保留原文件并补充新变量
cp .env.example .env
```

主要配置如下（完整默认值以 `.env.example` 为准）：

| 变量 | 新配置示例 | 说明 |
|---|---|---|
| `ROBOT_PROFILE` | `chassis_3d_ur5e` | 默认整机；`chassis_3d` 为纯底盘 |
| `ROBOT_TYPE` | `omni_3wd` | 当前部署要求三舵轮 |
| `ROBOT_PACKAGE` | `simulated_chassis` | 纯底盘 bridge 的机器人配置包；组合入口内部固定此值 |
| `SLAM_ALGORITHM` | `lightning` | 可改为 `cartographer` |
| `PBSTREAM_FILE` | 留空 | 初始定位地图，路径必须是容器内路径，见下节 |
| `MANAGE_NAVIGATION` | `true` | bridge 托管定位/建图/Nav2 |
| `AGV_ID` | `AGV001` | 机器人标识 |
| `ROS_DOMAIN_ID` | `1` | 两容器相同，区别于 WSL 原生机器人的 domain 0 |
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | 当前 DDS 配置使用 Cyclone DDS |
| `USE_SIM_TIME` | `true` | 仿真时间；UR5e 组合入口内部固定为 true |
| `BUILD_JOBS` | `2` | 镜像内编译并行度；内存不足可改为 1 |
| `GAZEBO_GUI` / `START_RVIZ` | `false` | 默认无界面运行 |
| `WEBSOCKET_PORT` | `9090` | 宿主 WebSocket 端口；容器监听固定 9090 |
| `MAP_3D_PORT` | `8089` | 三维地图服务的宿主端口；容器 8089 |
| `URDF_MESH_PORT` | `8090` | 模型资源服务的宿主端口；容器 8090 |
| `BACK_UP_MAX_HEADING_ERROR_DEG` | `20.0` | 倒车允许的最大航向误差 |

新 `.env.example` 显式选择 Lightning。为兼容已有部署，旧 `.env` 若未设置
`SLAM_ALGORITHM`，Compose/启动脚本仍回退到 Cartographer。
不要仅替换镜像而遗漏这个变量。

```bash
docker compose config --quiet
bash deploy.sh build
bash deploy.sh up
bash deploy.sh status
bash deploy.sh logs agv-bridge
```

`build` 先构建 `ros2-base:latest`，再构建两个服务镜像，包含 Lightning、
按镜像 CPU 架构编译的 Pangolin、UR/MoveIt、机械臂接口和 OpenCV/cv_bridge。
`lightning-lm` 的 `COLCON_IGNORE` 由专用构建脚本临时处理，不能只追加普通
`colcon --packages-select lightning`。首次编译较慢，后续可复用依赖缓存层。
amd64/arm64 应分别构建对应架构镜像；宿主构建产物不会复制进镜像。

新增源码依赖或修改镜像内模型/配置后重新 `build`，再执行
`bash deploy.sh restart` 重建运行容器。不要混用旧基础镜像与新服务源码。
Windows 透明加密软件可能使文件在 Windows 可读、在 WSL/Docker 不可读；
构建脚本会检查已知加密文件签名，构建源码应保持 Linux 可读明文。

## 4. 建图、保存与定位

`PBSTREAM_FILE` 名称为历史兼容参数，含义由算法决定：

| 算法 | `.env` 示例 | 主要地图文件 |
|---|---|---|
| Lightning | `PBSTREAM_FILE=/ros2_ws/maps/lm_map` | 地图目录内的分块点云、`global.pcd`、`map.pgm`、`map.yaml` |
| Cartographer | `PBSTREAM_FILE=/ros2_ws/maps/carto_map/carto_map.pbstream` | `.pbstream`、PGM、YAML |

两类地图不能相互替代。宿主机 `maps/` 挂载到 `/ros2_ws/maps`，不要填写宿主机
Windows 路径。初始地图路径非空但不存在时，bridge 启动脚本会报错退出。

留空时启动后尚无定位和 Nav2；从前端发起建图，或执行：

```bash
bash deploy.sh start-mapping
# 通过前端驾驶机器人采集环境，然后保存
bash deploy.sh save-map lm_map
```

建图、保存请求是异步操作：`success=true` 表示受理，不表示地图已保存完成。
查看 `/agv/status`、bridge 日志和生成文件；保存完成后 bridge 切回定位，
定位 TF 就绪后启动新的 Nav2。将成功保存的地图路径填入 `.env`，
执行 `bash deploy.sh restart` 后，后续启动即可加载该地图。
Lightning 当前定位接入不保证任意初始位置的全局重定位，首次默认靠近建图起点。

`up-nav` 是 `up` 的别名；`up-slam` 启动同样两个容器后调用建图服务，
不是增加第三个容器。服务命令最多等待 60 秒（包含 ROS 发现时间）；
若启动初期尚未就绪，先检查日志和状态，再调用 `start-mapping`。

## 5. 机械臂与腕部相机

### UR5e 运行时

仿真容器生成一台底盘 + UR5e 整机；bridge 在就绪检查后启动 MoveIt、
`arm_manager` 和 C++ `arm_move_adapter`。无需另外启动单臂仿真或管理器。

| 接口 | 职责 |
|---|---|
| `/joint_states` | 整机实际关节状态，供上位机模型联动 |
| `/agv/arm_move` | 兼容上层关节目标协议，适配为 JTC 轨迹；受理即返回 |
| `/arm_manager/command` Action | ROS 2 原生的 MoveIt 规划与执行任务；远端接入需另外核对 rosbridge 配置 |
| `/arm_manager/status` | 管理器状态 |
| `/arm_manager/save_teach_point`、`list_teach_points`、`delete_teach_point` | 示教点管理服务（均带 `/arm_manager/` 前缀） |

`/agv/arm_move` 的 `left`/`right` 是同一个六轴 UR5e 的协议别名，
不是两条实体机械臂；`waist` 未装备。该服务不经过 MoveIt 避碰规划，
成功响应不代表物理到位。关节顺序、6/7 轴映射和连续点动限制见
[机械臂对接文档](docs/arm_move_integration.md)。
两条命令链路共享执行器，不应假设它们已有统一仲裁。

安装位置来自 `src/simulated_chassis_ur5e/config/mounting.yaml`，
初始折叠角度来自 `src/simulated_ur5e/config/initial_positions.yaml`。
修改模型后重建镜像并重启两容器，保证仿真与 MoveIt 描述一致。

### 相机与拍照

相机由仿真容器启动：Gazebo 渲染图像，经 `ros_gz_bridge` 转 ROS 消息，
再由 `wrist_camera_node` 统一光学 frame 并提供拍照服务。
配置为 `src/simulated_chassis_ur5e/config/wrist_camera.yaml`，
默认启用，640×480、10 Hz。仿真相机无需挂载宿主 USB 设备。

| 接口 | 类型/含义 |
|---|---|
| `/wrist_camera/image_raw` | `sensor_msgs/msg/Image`，sensor-data QoS |
| `/wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo` |
| `/wrist_camera/take_photo` | `std_srvs/srv/Trigger`，等待请求后的新帧并保存 |

组合 bridge 已在 rosbridge 白名单中放行上述话题和拍照服务。
拍照成功后 `message` 返回仿真容器内 PNG 路径，每次拍照创建独立目录，
包含 `image.png`、`metadata.json`；这个文件路径不是浏览器下载 URL，
8090 也不是照片文件服务器。照片可从下节的宿主挂载目录取出。

当前传感器持续出图，但只有调用 take_photo 才保存PNG；尚无照片HTTP下载接口。
请求/响应、预览、元数据和末端按需拍照流程见 [相机接口文档](docs/camera_api.md)。

仿真和机械臂运行时须使用相同相机模型配置。
`camera_file`、`photos_dir` 是 launch 参数，当前 Docker 包装脚本未直接暴露为 `.env`
变量；自定义应修改镜像配置并重建，或同时扩展两端挂载和启动参数。
详细模型及相机坐标约定见 [组合包说明](src/simulated_chassis_ur5e/README.md)。

## 6. 持久化目录与日志

| 宿主路径/卷 | 容器路径 | 内容 |
|---|---|---|
| `maps/` | bridge `/ros2_ws/maps` | 两种后端的地图、三维导出缓存及记忆位姿等 |
| `runtime/bridge/ros/` | bridge `/root/.ros` | ROS 日志、机械臂示教点 |
| `runtime/bridge/process/` | bridge `/ros2_ws/process_logs` | 定位、导航和建图子进程日志 |
| `runtime/simulation/ros/` | simulation `/root/.ros` | 仿真 ROS 日志、相机照片 |
| Compose 命名卷 `simulation_data` | simulation `/root/.ignition` | Gazebo 运行数据 |

关键文件：

- 示教点：`runtime/bridge/ros/arm_manager/mobile_ur5e_teach_points.json`。
- 照片：`runtime/simulation/ros/wrist_camera/photos/<每次拍照目录>/`。
- 子进程日志：`agv_localization.log`、`agv_navigation.log`、`agv_slam.log`；
  位于 `runtime/bridge/process/`，每次对应子进程启动会覆盖该文件。

`bash deploy.sh down` 保留这些数据；源码更新和镜像导出均不包含运行数据。
迁移前单独备份 `maps/`、`runtime/` 和 `.env`。

## 7. 离线镜像迁移

```bash
# 构建机器
bash deploy.sh build
bash deploy.sh export dist/simulated-chassis-images.tar

# 同 CPU 架构的目标机器，在部署目录中
bash deploy.sh load dist/simulated-chassis-images.tar
docker compose config --quiet
docker compose up -d --no-build --pull never
```

tar 包含三个镜像。目标部署目录至少需要 `deploy.sh`、`docker-compose.yml`、
`docker/cyclonedds.xml` 和按目标机器配置的 `.env`；离线启动不需要完整源码，
但需要 Docker 和 Compose。需要恢复的地图、示教点、照片另外复制到相应目录。
计划在目标机器重新构建时则需完整仓库和可用的构建依赖下载源。

## 8. 网络、图形与运行检查

DDS 默认使用 Cyclone DDS，Compose 网络为 `172.28.0.0/16`，固定 IP
`.10`、`.11` 与 `docker/cyclonedds.xml` 的 peers 一致。网段冲突时需一起修改
Compose 子网、两容器 IP 和 XML peers。宿主 ROS 节点不自动加入这套 DDS 网络，
调试时优先进入容器。

默认 `GAZEBO_GUI=false`，仿真容器使用 Xvfb 和软件渲染为 GPU 雷达/相机提供图形上下文。
打开 GUI/RViz 还需配置可访问且获授权的 X11 display；仅改开关不等于完成显示转发。

```bash
bash deploy.sh status
bash deploy.sh logs robot-simulation
bash deploy.sh logs agv-bridge
bash deploy.sh shell agv-bridge
```

进入容器后先加载环境（另开 `docker compose exec` 命令时同样需要）：

```bash
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /clock
ros2 topic hz /points2_1
ros2 topic echo /joint_states --once
ros2 service list
ros2 pkg executables lightning
# 定位就绪后才检查
ros2 topic hz /tracked_pose
```

相机检查请进入仿真容器：

```bash
bash deploy.sh shell robot-simulation
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
ros2 topic hz /wrist_camera/image_raw
ros2 topic echo /wrist_camera/camera_info --once --qos-reliability best_effort
ros2 service call /wrist_camera/take_photo std_srvs/srv/Trigger '{}'
```

从能访问机器人主机的机器检查 HTTP（地图名称换成实际已保存的地图）：

```bash
curl -I http://HOST:8090/ur_description/meshes/ur5e/visual/base.dae
curl http://HOST:8089/maps/lm_map/status
# status 为 ready 后下载
curl -o lm_map.ply http://HOST:8089/maps/lm_map/cloud.ply
```

| 现象 | 检查方向 |
|---|---|
| WebSocket 正常但模型缺失 | `robot_description` 参数读取白名单、前端 mesh base、8090→8090 映射及具体资源 URL |
| 三维地图返回 409 | 等待导出，查看 status；确认地图保存完成和所选 SLAM 来源一致 |
| bridge 因地图不存在退出 | `.env` 的容器内路径、`maps/` 挂载、地图后端是否匹配 |
| 机械臂节点没有启动 | bridge 的就绪检查日志、模型一致性、JTC 是否 active、DDS 连接 |
| 相机无图像 | 是否使用 UR5e profile、相机 enabled、Xvfb/渲染和仿真日志 |
| 参数修改后未生效 | 环境变量修改需 `restart` 重建容器；镜像内文件修改还需 `build` |

当前脚本命令以 `bash deploy.sh` 的 usage 为准：`logs [SERVICE]` 自带 `-f`，
例如 `bash deploy.sh logs agv-bridge`；脚本没有 `clean` 命令。

## 9. 进一步阅读

- [复合机器人架构与 C++ HTTP 服务](docs/composite_robot_architecture.md)
- [建图、定位与导航操作](docs/mapping_and_localization.md)
- [3D 地图数据与 HTTP 接口](src/agv_map_3d/README.md)
- [上位机 rosbridge 对接](docs/rosbridge_integration.md)
- [机械臂接口契约](docs/arm_move_integration.md)
- [UR5e 与腕部相机](src/simulated_chassis_ur5e/README.md)
