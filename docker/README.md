# Docker 构建与部署

完整部署说明统一维护在 [DOCKER_DEPLOY.md](../DOCKER_DEPLOY.md)，包括：

- 双容器进程架构及 Lightning/Cartographer 的选择。
- 9090 rosbridge、8089 三维地图服务、8090 URDF mesh/纹理服务。
- URDF 文本读取与模型资源下载的区别。
- UR5e、MoveIt、机械臂适配接口、腕部相机和拍照。
- 构建启动、离线镜像迁移、数据备份、运行检查及故障排查。

本目录中的实现入口：

| 路径 | 用途 |
|---|---|
| `ros2-base/Dockerfile` | ROS 2 Humble、Gazebo、Nav2、Cartographer、UR/MoveIt 和通用依赖 |
| `simulated-robot/Dockerfile` | 仿真、模型、控制器和相机镜像 |
| `simulated-robot/launch_robot.sh` | 按机器人 profile 启动物理仿真，无界面时使用 Xvfb |
| `agv-bridge/Dockerfile` | Lightning/Pangolin、桥接、地图服务和机械臂运行时镜像 |
| `agv-bridge/launch_bridge.sh` | 传递算法、地图和运行参数，启动对应 bridge 入口 |
| `cyclonedds.xml` | 与 Compose 两容器固定 IP 配套的 DDS 发现配置 |

从仓库根目录运行 `bash deploy.sh build` 构建三个镜像，
运行 `bash deploy.sh up` 启动两个服务。配置以根目录
[`docker-compose.yml`](../docker-compose.yml) 和 [`.env.example`](../.env.example) 为准。
