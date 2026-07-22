# AGV机器人仿真系统 Docker部署

## 架构说明

```
┌─────────────────────────────────────────────────────────────┐
│                    Docker Network (172.28.0.0/16)            │
│                                                             │
│  ┌──────────────┐  ┌─────────────────┐                     │
│  │ agv-bridge   │◄─┤ robot-simulation │                     │
│  │ :9090 (WS)   │  │ :11345 (Gazebo)  │                     │
│  │              │  │                  │                     │
│  │ ROS2 Bridge  │  │ Gazebo + 模型    │                     │
│  └──────┬───────┘  └────────┬─────────┘                     │
│         │                    │                              │
│         ▼                    ▼ (二选一)                      │
│   ┌───────────────────────────────────┐                    │
│   │     host.docker.internal          │                    │
│   │  ┌────────────┐ ┌──────────────┐  │                    │
│   │  │Spring Boot  │ │ Redis/MQTT   │  │  ← 上位机项目部署  │
│   │  │ :22777      │ │              │  │                    │
│   │  └────────────┘ └──────────────┘  │                    │
│   └───────────────────────────────────┘                    │
│                                                              │
│   可选服务（profile启动）:                                   │
│   ┌─────────────────┐  ┌─────────────────┐                 │
│   │  nav2-server    │  │  cartographer   │                 │
│   │  (导航)         │  │  (SLAM建图)     │                 │
│   └─────────────────┘  └─────────────────┘                 │
└─────────────────────────────────────────────────────────────┘
```

## 快速开始

### 1. 环境准备

```bash
# 安装Docker
sudo apt-get update && sudo apt-get install -y docker.io docker-compose-v2
sudo usermod -aG docker $USER
# 重新登录生效
```

### 2. 配置

```bash
# 编辑.env文件，修改上位机连接地址等配置
vim .env
```

**关键配置项：**
```bash
ROBOT_TYPE=diff_drive        # diff_drive(差速) 或 omni_3wd(三舵轮)
AGV_ID=AGV001               # 机器人ID
SPRINGBOOT_HOST=host.docker.internal  # 上位机地址
SPRINGBOOT_PORT=22777       # 上位机端口
```

### 3. 构建镜像

```bash
./deploy.sh build
```

### 4. 启动服务

```bash
# 基础模式：仿真 + Bridge
./deploy.sh up

# 导航模式：仿真 + Bridge + Nav2导航
./deploy.sh up-nav

# 建图模式：仿真 + Bridge + Cartographer SLAM
./deploy.sh up-slam
```

## 服务说明

| 服务 | 说明 | Profile |
|------|------|---------|
| robot-simulation | Gazebo机器人仿真 | 默认 |
| agv-bridge | ROS2↔HTTP/WebSocket桥接 | 默认 |
| nav2-server | Nav2导航服务器 | navigation |
| cartographer | Cartographer SLAM建图 | slam |

## 常用命令

```bash
./deploy.sh status                # 查看状态
./deploy.sh logs -f agv-bridge    # 跟踪Bridge日志
./deploy.sh restart               # 重启
./deploy.sh down                  # 停止
./deploy.sh clean                 # 清理所有资源
```

## 上位机对接说明

本项目的Bridge节点通过环境变量连接上位机，需在上位机的docker-compose.yml中：

```yaml
# 上位机compose中添加网络配置
services:
  springboot-app:
    networks:
      - agv_network  # 与机器人同一网络，或使用host网络

networks:
  agv_network:
    external: true
    name: agv_robot_agv_network
```

**或者使用host网络模式**（推荐简化方案）：
- 本项目compose使用 `extra_hosts: host.docker.internal:host-gateway`
- Bridge通过 `host.docker.internal` 访问宿主机上的上位机服务
- 上位机直接监听宿主机端口即可

## 开发调试

```bash
# 进入容器
docker exec -it robot_simulation bash
docker exec -it agv_bridge bash

# 查看ROS2话题
ros2 topic list
ros2 topic echo /scan --window 10

# 手动发布目标点
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0}}}"
```

## 目录结构

```
simulated_chassis/
├── docker/
│   ├── ros2-base/           # ROS2基础镜像 (Jazzy+Nav2+Carto)
│   ├── simulated-robot/     # 机器人仿真镜像
│   └── agv-bridge/          # Bridge节点镜像
├── src/
│   ├── jzt_robot/           # 2D差速底盘
│   ├── simulated_chassis/   # 3D三舵轮底盘
│   └── agv_bridge_v2/       # Bridge节点源码
├── maps/                    # 地图文件
├── nav2_params/             # Nav2参数
├── carto_config/            # Cartographer配置
├── docker-compose.yml
├── .env
└── deploy.sh
```
