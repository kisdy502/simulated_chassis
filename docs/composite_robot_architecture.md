# 复合机器人架构与 C++ HTTP 服务迁移

本说明依据 `dev-lightning` 分支源码。SLAM 后端由 `slam_algorithm` 选择，组合包的
launch 默认仍为 `cartographer`；使用 Lightning 时须显式传 `slam_algorithm:=lightning`。
这是一套可切换后端的架构，并没有删除 Cartographer。

## 1. 包职责

| 包 | 职责 | 主要实现 |
|---|---|---|
| `simulated_chassis` | 三舵轮控制插件、底盘与传感器模型、仿真启动、Nav2、点云合流、TF 重锚定、两套 SLAM 启动配置 | C++、Xacro、YAML、Python launch |
| `simulated_ur5e` | UR5e 单臂调试入口与初始关节配置 | Xacro、YAML、Python launch |
| `simulated_chassis_ur5e` | 底盘与 UR5e 的装配层，合并描述/控制器/MoveIt 参数，启动机械臂运行时 | Xacro、YAML、Python 编排、C++ arm_move_adapter |
| `simulated_dual_arm` | 底盘与双七轴臂、两轴腰的另一种组合；目前上装为视觉和关节插值仿真 | Xacro、Python 节点 |
| `simulated_arm` | 另一套独立六轴机械臂演示模型，不是 UR5e 组合链路的必经包 | Xacro、配置、launch |
| `arm_manager` / `arm_manager_interfaces` | MoveIt 运动任务、取消、状态、示教点和接口 IDL | 当前管理节点为 Python |
| `agv_bridge_v2` / `agv_bridge_v2_interfaces` | 对外导航/地图服务、任务状态机、SLAM/Nav2 生命周期与接口 IDL | 核心业务 C++；进程包装脚本和 launch 仍有 Python |
| `lightning-lm`（ROS 包名 `lightning`） | 激光惯性建图、回环、地图存储、定位 | C++，含本项目的 ROS 接口补丁 |

复合机器人没有同时启动“完整底盘机器人”和“完整单臂机器人”。它装配出一个模型，
由一个 Gazebo 世界和一个 controller_manager 管理。

## 2. 为什么原包不用大改

### 模型组合

`simulated_chassis/urdf/three_wheel_chassis_3d.xacro` 本身已经是底盘宏的薄包装。
`simulated_chassis_ur5e/urdf/robot.urdf.xacro` 复用同一个宏，再调用上装宏：

```xml
<xacro:three_wheel_chassis_3d controllers_file="..." lidar_collisions="true"/>
<xacro:mounted_ur5e mounting_file="..." sim_ignition="..."/>
```

`arm_mount.xacro` 创建带质量、惯量和碰撞几何的安装座，以固定关节接到
`base_link`，再调用系统 `ur_description` 的官方 `ur_robot` 宏。
`ur5e_` 前缀用于避免关节与 link 重名；位置、尺寸、质量来自 `mounting.yaml`。

因此 UR5e 组合包主要直接复用官方 UR 模型；并不是把 `simulated_ur5e` 的独立
launch 再启动一次。独立包在这里主要提供共用初始角度和单臂调试能力。

### 配置组合

`simulated_chassis_ur5e/configuration.py::merged_controllers()` 读取原底盘控制器
YAML，再叠加上装的 `joint_trajectory_controller`。底盘调参仍只维护一份。

`simulation.launch.py` 临时生成合并后的控制器 YAML 和 URDF，调用原底盘
`three_wheel_sim.launch.py`，传入 `description_file`、`rsp_description_file` 和
`extra_controllers`。底盘 launch 已提供这些扩展点，因此不必复制完整仿真流程。

整机运行时为：

```text
Gazebo /clock + 一个整机模型
  └─ 一个 Ignition ros2_control 插件 / controller_manager
       ├─ 底盘硬件组件 → three_wheel_base_controller
       ├─ UR5e 硬件组件 → joint_trajectory_controller
       └─ joint_state_broadcaster → /joint_states
                                       └─ robot_state_publisher → 整机 TF
```

物理仿真描述使用 `file://` mesh 路径。给浏览器读取的 RSP 描述保持同样的仿真
硬件定义，仅把 mesh 前缀改成 `package://`；不能简单切换官方宏的仿真开关，
否则会同时切换硬件插件。

### 运行时组合

`bridge.launch.py` 包含原通用 AGV bridge，再包含 `arm_runtime.launch.py`。
机械臂就绪门检查通过后，才启动 MoveIt、arm_manager 和机械臂接口适配节点。
仿真进程与业务运行时分开，重启 bridge 不需要重新生成物理机器人。

MoveIt 的整机 SRDF 将底盘关节标记为 passive，机械臂规划可以考虑车体和雷达的
模型碰撞，但不会规划底盘运动。当前是底盘导航与机械臂运动的组合系统，尚不是
底盘和机械臂同时联合求解轨迹的移动操作规划器。

## 3. Lightning 如何替换 Cartographer

```text
/points2_1 + /points2_2
          ↓ dual_lidar_merge_node（外参 TF、合流、字段/时间适配）
/points2_merged（base_link） + /imu
          ↓ lightning
          ├─ run_slam_online：LIO、回环、g2p5 栅格、存图
          └─ run_loc_online：加载分块点云地图定位
          ↓ /tracked_pose（map 中的 base_link 位姿）
tracked_pose_tf_node + 同时刻的底盘 TF
          ↓ map → odom
底盘控制器 → odom → base_footprint → base_link
```

`lightning_sim_3d.yaml` 是传感器、LIO、定位和栅格参数的适配层。
快照式仿真点云使用合流节点的 `time_mode=zero`，而不是伪造真实旋转雷达扫描时间。

本项目对 Lightning 的 `core/system/slam.cc`、`loc_system.cc` 做了适配：
建图/定位发布 `/tracked_pose`；建图发布 `/map`；定位配置 `pub_tf=false`。
`tracked_pose_tf_node` 按相同时间戳计算：

```text
T_map_odom(t) = T_map_base_link(t) × inverse(T_odom_base_link(t))
```

这样 SLAM 后端只负责全局定位，底盘控制器继续独占局部里程计 TF。
Nav2 消费的 `/map` 与 TF 契约保持一致，可以复用原规划、控制、速度平滑链路。

地图保存有所不同：Cartographer 使用 pbstream；Lightning 使用地图目录中的
分块点云、`global.pcd`、`map.pgm`、`map.yaml`。Bridge 的后端分支负责适配
启动参数、存图服务、文件检查与定位就绪判定。历史参数名 `pbstream_file` 在
Lightning 模式下也可传地图目录；名称沿用并不代表 Lightning 读取 pbstream。

定位阶段由 `nav2_map_server` 提供静态 `/map`。初始位姿通过 init 参数注入，
当前接入链路不能理解为支持任意位置的可靠全局重定位。

## 4. 两条机械臂命令链路的区别

```text
/arm_manager/command Action
  → arm_manager → MoveGroup（规划）→ ExecuteTrajectory（执行）
  → joint_trajectory_controller → UR5e 仿真硬件

/agv/arm_move Service
  → arm_move_adapter → /joint_trajectory_controller/joint_trajectory
  → joint_trajectory_controller → UR5e 仿真硬件
```

第一条有管理器任务语义和 MoveIt 规划。第二条是受理即响应的关节轨迹适配，
不经过 MoveIt 避碰规划；不能把两条链路当作同等的执行确认或避碰保证。
二者共享执行器，也不能把“各自能接收指令”当作具备统一仲裁。

双臂组合目前与此不同：`dual_arm_sim_node.py` 插值并合并 `/joint_states`，
没有上装 JTC/MoveIt/物理关节控制链路，不能以 UR5e 的物理仿真能力来推断双臂能力。

## 5. URDF 资源 HTTP 服务的 C++ 替换

URDF 文本来自 `robot_state_publisher` 的 `robot_description` 参数，经 rosapi
供上位机读取。8090 服务提供的是 URDF 引用的 mesh、纹理等包资源，不生成 URDF，
也不参与 SLAM、TF 或运动控制。

```text
package://ur_description/meshes/ur5e/visual/base.dae
  → 浏览器重写为 http://<机器人>:8090/ur_description/meshes/ur5e/visual/base.dae
  → ament_index_cpp 查找 package share
  → Boost.Beast file_body 发送文件
```

替换后的实现是 `agv_bridge_v2/src/urdf_mesh_server.cpp`：

- C++17；HTTP 协议、异步网络与文件响应使用 Boost.Beast/Asio。
- 显式依赖 `boost`（Ubuntu 开发包 `libboost-system-dev`）和已有的
  `ament_index_cpp`。不使用 TLS，无需为本服务增加 OpenSSL 配置或独立 Web 容器。
- 保留 `--host`、`--port`、默认 0.0.0.0:8090 和原资源 URL。
- 支持 GET、HEAD、OPTIONS；跨域响应包含 `Access-Control-Allow-Origin: *`。
- 文件响应使用 `file_body`，避免像旧实现一样将完整大 mesh 读入字符串。
- 单个异步事件循环处理多连接；读写超时 30 秒，限制请求头/请求体大小。
- URL 解码后拒绝 `..`、控制字符和反斜线；不列目录；未知包/文件返回 404。
- 保留 symlink-install 资源访问。已安装包与其中的符号链接视为受信任资源；
  这不是允许不可信用户上传文件的通用文件服务器。
- SIGINT/SIGTERM 结束事件循环；端口占用或非法参数启动失败并返回非零退出码。
- UR5e 与双臂两个 bridge 入口均改为从 ament 可执行目录启动同一个 C++ 程序。

构建与验证（在已安装工作空间依赖的 WSL 环境）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
# 仅在依赖尚未安装时需要：sudo apt-get install libboost-system-dev
colcon build --symlink-install --packages-select agv_bridge_v2 simulated_chassis_ur5e simulated_dual_arm
source install/setup.bash
colcon test --packages-select agv_bridge_v2 --ctest-args -R test_urdf_mesh_server --output-on-failure
colcon test-result --verbose
ros2 run agv_bridge_v2 urdf_mesh_server --host 127.0.0.1 --port 18090
```

HTTP 测试使用独立临时 ament 包、符号链接和高位端口，验证真实网络访问、编码路径、
查询参数、HEAD/CORS、穿越拒绝、并行二进制文件传输、参数错误、端口冲突和退出。
Docker 基础镜像也显式安装该 Boost 开发依赖；镜像内置旧程序需重新构建才会更新。

## 6. 开发语言约定与迁移边界

新增业务功能与独立运行服务采用 C++17，ROS 节点使用 rclcpp，沿用现有包、
IDL、命名和构建方式；通用协议交给成熟库，避免手写 HTTP 或自制任务框架。
launch、Xacro、YAML 属于 ROS 启动与配置层，可以保留。Python launch 不要求
业务节点也使用 Python，不能通过迁移 HTTP 服务就声称项目已全量 C++ 化。

现存业务 Python 仍包括 arm_manager、dual_arm_sim_node；arm_move_adapter 已迁移 C++，
另外还有就绪检查和进程包装脚本。本次只替换共享 HTTP 服务，后续若迁移这些节点，
应逐个保留 IDL、任务取消/超时/执行完成语义，并单独回归；机械臂运动状态机的
迁移难度明显高于静态文件服务，不宜与本次 HTTP 改动捆绑重写。
