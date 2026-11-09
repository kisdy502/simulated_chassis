# 3D 地图服务

完整请求参数、响应字段、状态码与 URDF/模型接口联调见
[当前接口契约（2026-10-23）](../../docs/map_model_api.md)。

机器人侧生产功能使用 C++17：`robot_http_server`（统一 Boost.Beast/Asio HTTP 路由、ROS 实时地图订阅和地图导出任务）、`pbstream_to_ply`（Cartographer）、`pcd_to_ply`（PCL）。Python 仅用于测试；ROS Python launch 保留为启动配置。

## 数据链路与边界

- Cartographer：`maps/name/name.pbstream` 或旧式 `maps/name.pbstream` → 优化后的子图位姿转换三维占据体素 → XYZ PLY。
- Lightning：`maps/name/global.pcd` → PCL 体素质心降采样 → `global.ply`。
- bridge 按当前 `slam_algorithm` 明确选择来源，避免同名旧地图覆盖当前后端；独立服务的 `--source-format auto` 保留嵌套 PBStream、扁平 PBStream、PCD 的优先顺序。
- 坐标为 ROS `map`，米，Z 向上。默认体素 0.1 米，最多 500 万点。二维地图无法凭空恢复高度。
- PGM/YAML、Nav2 与原有二维页面继续保留。这是**保存地图的三维点云查看**，不是实时建图直播、网格重建或三维导航。

## 构建、启动

依赖 Cartographer、PCL、Boost system/filesystem、nlohmann_json、Threads；Ubuntu 对应开发包包含 `libpcl-dev libboost-system-dev libboost-filesystem-dev nlohmann-json3-dev`。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_map_3d agv_bridge_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py maps_dir:=/absolute/path/maps
```

统一 HTTP 默认端口 8080；`http_port:=8080` 改端口，`enable_map_3d:=false` 仅关闭 3D 路由和导出后台，其他 HTTP 路由仍共享监听器。`mesh_server:=false` 关闭 mesh 路由，`enable_map_2d_http:=false` 关闭实时地图路由。需要重新构建并在下次启动 bridge 时加载。服务不发送机器人控制指令。

独立运行：

```bash
ros2 run agv_map_3d robot_http_server --maps-dir /absolute/path/maps --port 8080 --source-format pcd
ros2 run agv_map_3d pcd_to_ply input.pcd output.ply 0.1
ros2 run agv_map_3d pbstream_to_ply input.pbstream output.ply 0.1 0.55
```

## API 与任务处理

```bash
curl http://localhost:8080/maps/map1008Pro/status
curl -o map1008Pro.ply http://localhost:8080/maps/map1008Pro/cloud.ply
```

状态 JSON 的 `status` 为 `generating`、`ready` 或 `failed`，包含地图名、`frame_id`、版本、体素大小，完成后包含点数、字节数；失败含 `message`。下载未就绪返回 409，地图不存在返回 404。支持 GET/HEAD/OPTIONS 与 CORS。PLY 为二进制 little-endian float XYZ。

服务启动即扫描，之后每 10 秒扫描；HTTP 查询也会触发检查。源文件签名需观察稳定至少 2 秒，导出后再次检查版本，成功才原子替换最终 PLY 和 `.3d.json`。这些检查减少读取保存中间态的风险，**不能代替保存完成信号**：请在保存地图成功后请求。Lightning 保存会重建目录，尤其需要遵守这一顺序。

导出串行后台执行，不阻塞 HTTP；默认 300 秒超时，失败 30 秒后允许重试。源版本改变自动重新生成。退出或超时会结束当前导出进程组。源文件、输出文件均检查根目录和符号链接边界；同一根目录通过文件锁禁止重复服务。旧缓存需通过 PLY 格式和长度检查才会返回 ready。

## Docker 与上位机

Docker 构建已加入本包及依赖，bridge 容器内部固定 8080，Compose `HTTP_PORT` 设置宿主机端口（默认 8080）。同 Docker 网络的后端使用 `http://agv-bridge:8080`，宿主机/浏览器直连使用 `http://HOST:8080`。WSL 与 Docker 仿真交替运行，避免标准端口被同时占用。详见[部署网络与端口说明](../../DOCKER_DEPLOY.md)。端口映射更改后需重建容器，无需仅为映射重编译镜像。

上位机 Java 代理的 HTTP path 由上位机项目管理；机器人侧统一 base URL 为 `http://<robot>:8080`。path、HTTP method、认证头、请求与响应格式不变，上位机连接配置只需改为统一端口。Three.js 保留 Z 向上，按高度着色，支持缩放旋转。二维入口继续可用；机器人的页面箭头只表示平面位置和 yaw，不表示完整 roll/pitch/Z。

## 验证与已修复问题

```bash
colcon test --packages-select agv_map_3d --event-handlers console_direct+
colcon test-result --verbose
```

`BUILD_TESTING=ON` 使用 Python 驱动 C++ 可执行文件：坐标转换测试、PCD 导出测试、HTTP 黑盒测试。坐标夹具含非零局部位姿、全局旋转、重叠子图和自由体素，期望输出唯一 `(10,21,5)`。

本次修复：Lightning 自动扫描误用 global 作为地图名；PBStream 符号链接越界；异常遗留 generating；失败不能自动恢复；源更新期间发布旧结果；退出后导出子进程残留；同名不同后端地图混用；PCL 1.12 的 const reader 编译错误及索引溢出保护；Docker 漏构建与端口映射不一致。HTTP 测试覆盖并发去重、缓存损坏、超时、进程树清理、只读目录、路径边界与后端选择。
