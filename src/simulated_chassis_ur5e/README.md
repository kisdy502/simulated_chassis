# 三舵轮 3D 雷达 + UR5e 整机仿真

## 末端 RGB 相机（新增）

整机默认在 `ur5e_tool0` 安装 RGB 相机（eye-in-hand）。实现位于本包，不修改官方 UR 模型；相机沿用现有 Fortress Sensors 渲染系统，通过独立单向 `ros_gz_bridge` 接入 ROS。Lightning 与 3D 地图继续使用原雷达数据链路。

`urdf/wrist_camera.xacro` 定义带惯量、碰撞体的相机外壳及光学坐标系；`config/wrist_camera.yaml` 配置开关、安装位姿和画面参数。默认 640×480、10 Hz、水平视场约 60°、裁剪距离 0.02～20 m。橙色外壳 50×50×35 mm、质量 0.10 kg；绿色实体支撑杆直径 20 mm、长度 40 mm、质量 0.03 kg，两者按同一刚体建模并包含碰撞几何。默认视线沿 tool0 的 +Z，外壳中心距离 tool0 为 0.065 m，镜头位于外壳 X=0.025 m。默认支撑杆从 tool0 连至相机背面；修改安装位姿时须同步调整支撑杆几何以保持连接，修改实体规格时须同步修改惯量和碰撞体。

TF 链为 `ur5e_tool0 → ur5e_camera_link → ur5e_camera_optical_frame`。光学坐标 Z 向前、X 向右、Y 向下。Fortress 的 scoped sensor frame 经 `wrist_camera_node` 统一为此光学 frame。原规划 TCP 仍是 `ur5e_tool0`，相机不会自动成为 MoveIt 目标坐标。

| 公共接口 | 类型/用途 |
|---|---|
| `/wrist_camera/image_raw` | `sensor_msgs/msg/Image`，sensor-data QoS |
| `/wrist_camera/camera_info` | `sensor_msgs/msg/CameraInfo`，渲染相机内参 |
| `/wrist_camera/take_photo` | `std_srvs/srv/Trigger`，等待请求后的新帧并保存 PNG |

```bash
# Ubuntu/WSL Humble 环境，首次安装依赖
sudo apt-get install ros-humble-cv-bridge ros-humble-ros-gz-bridge ros-humble-std-srvs python3-opencv
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select simulated_chassis_ur5e
source install/setup.bash

# 终端一，无显示器时外层加 xvfb-run -a
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true photos_dir:=$HOME/robot_photos
# 终端二，可继续附加现有建图/定位参数
ros2 launch simulated_chassis_ur5e bridge.launch.py slam_algorithm:=lightning
# 终端三，停车并确认机械臂实际到位后拍照
ros2 service call /wrist_camera/take_photo std_srvs/srv/Trigger '{}'
ros2 topic hz /wrist_camera/image_raw
ros2 topic echo /wrist_camera/camera_info --once --qos-reliability best_effort
ros2 run tf2_ros tf2_echo ur5e_tool0 ur5e_camera_optical_frame
```

RViz 增加 Image 显示，话题选择 `/wrist_camera/image_raw`，Reliability 选择 Best Effort。服务成功时 `message` 返回机器人侧 PNG 绝对路径；默认存储在 `~/.ros/wrist_camera/photos`。每次创建唯一子目录，包含 `image.png` 与 `metadata.json`。元数据保存图像仿真时间戳、frame_id、分辨率及最近一份匹配分辨率的内参；内参包含自己的时间戳，尚未收到时为 null，不声称两者严格同步。

服务默认最多等待 5 秒墙上时间，仅接受时间戳晚于请求的图像。无图像、仿真暂停或断流时返回 false，保存失败时清理该次目录。节点参数 `capture_timeout_sec` 可调整等待时间。此服务不等待机械臂到位，也未保存关节或全局位姿；后续用于测量/目标定位时应按图像时间戳获取并保存 TF，真机还需要手眼标定。仿真图像不模拟曝光、运动模糊、USB 延迟和真实镜头畸变。

自定义时复制 YAML，仿真和 bridge 必须同时传入同一 `camera_file:=/absolute/path/wrist_camera.yaml`。`enabled: false` 关闭相机。两端模型不一致时，现有 readiness 检查会拒绝启动机械臂 runtime。修改模型后重启仿真和 bridge。

rosbridge 已允许公共图像/内参订阅及拍照服务，上位机可通过原 WebSocket 调用 Trigger。PNG 保存在机器人/容器内，返回路径不是下载 URL，当前未增加 HTTP 下载或网页预览。Docker 已补充运行依赖，需要重建 simulated-robot 镜像，并将照片目录挂载到宿主机以持久保存。640×480×3×10 的原始图像约 9.2 MB/s，JSON/base64 传输还有额外开销；持续远程预览建议后续增加压缩图像或专用视频服务。

```bash
# 无帧超时、旧帧拒绝、新帧保存；不启动机器人
ROS_DOMAIN_ID=93 python3 src/simulated_chassis_ur5e/test/test_wrist_camera.py
# 真实 Fortress：图像/内参、两次拍照及原底盘、机械臂、碰撞回归
python3 src/simulated_chassis_ur5e/test/integration_fortress.py
```

本包只负责仿真组合：安装座、带 `ur5e_` 前缀的机械臂、组合碰撞/控制配置、启动编排。底盘算法继续在 `simulated_chassis`，机械臂业务在 `arm_manager`，单臂调试在 `simulated_ur5e`，通用/真机 bridge 保持独立。

## 上层真机协议与单臂仿真映射

`/agv/arm_move` 由 C++ `arm_move_adapter` 提供，服务类型仍为
`agv_bridge_v2_interfaces/srv/ArmMove`，JSON/IDL 字段不变。`left` 和 `right`
均控制同一个 UR5e，后来的请求从最新实测角度重新生成轨迹并替换前一个目标。
它们不代表仿真中存在两条独立机械臂。

上层可以继续发送真机每臂 7 个角度（joint_1..joint_7）。默认映射如下：

| 输入轴（从 1 开始） | UR5e 实体关节 |
|---|---|
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | 额外上臂回转不仿真，响应明确包含 `joint_3 not simulated` |
| 4 | elbow |
| 5 | wrist_1 |
| 6 | wrist_2 |
| 7 | wrist_3，保留末端回转 |

这是为上层协议联调提供的单臂别名映射，不是 7 轴机械臂的等效运动学。
索引配置见 `config/arm_move_adapter.yaml` 的 `real_arm_joint_indices`，
它在 UR5e 字典序中使用零起始索引，默认 `[3,1,0,4,5,6]`。
如改变配置，未被映射的输入轴会在响应中按实际索引提示。

兼容原 6 轴请求，其顺序为 UR5e **字典序**：
`elbow, shoulder_lift, shoulder_pan, wrist_1, wrist_2, wrist_3`。
字典序与机构顺序不同，调用端不可混用。数量为 6/7 以外或包含 NaN/Inf 时拒绝。
腰部未装备，`arm=waist`（即使给出 2 个角度）返回
`success=false, message="waist not equipped"`，不伪造腰部动作完成。

回调只做校验并下发两点 `JointTrajectory`，不会等待几秒运动结束。
JTC 在其控制周期内做线性插值，不需要适配节点再以 50Hz 发布角度。
`duration_sec<=0` 使用 3 秒；正时长限制为 0.5～300 秒。角度按当前整机 URDF
限位截断，必要时延长时间，使所生成线性参考轨迹不超过 URDF 关节速度限值。
这不提供加速度约束、MoveIt 避碰规划或物理到位保证。

接受请求前须收到 2 秒内完整、有限的实测 6 轴状态，并且控制器状态查询显示
`joint_trajectory_controller` 为 active、查询结果未过期、轨迹话题有订阅者；
未就绪/失联返回 false，不能用历史状态继续下发。
`success=true` 表示适配层已受理并发布，不表示控制器确认完成。
执行情况通过 JTC 日志和实际 `/joint_states` 观察。

上报保持实际 URDF 的 `ur5e_*` 六关节（以及原底盘关节），不额外发布虚构的
左臂、右臂或腰关节；不修改 ArmMove 响应去拼装虚拟状态。rosbridge 已放行
`/agv/arm_move` 和 `/joint_states`，上位机通过原链路驱动当前 URDF 模型。

```bash
# 7轴输入跳过额外上臂回转轴3，保留末端轴7
ros2 service call /agv/arm_move agv_bridge_v2_interfaces/srv/ArmMove \
  "{arm: left, joints: [0.5, -1.2, 0.0, 1.0, -0.5, 0.0, 0.0], duration_sec: 4.0}"

# 只有实际 UR5e 和底盘状态，不虚构 16 轴上装
ros2 topic echo /joint_states
```

隔离契约测试（ROS domain 88，假控制器，不移动机器人）：

```bash
colcon test --packages-select simulated_chassis_ur5e --ctest-args -R test_arm_move_adapter --output-on-failure
colcon test-result --verbose
```

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agv_bridge_v2_interfaces agv_bridge_v2 arm_manager_interfaces arm_manager simulated_chassis simulated_ur5e simulated_chassis_ur5e
source install/setup.bash

# 终端一：一个世界、一台组合机器人、一个 controller_manager
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true

# 终端二：原有 bridge + MoveIt + arm_manager，无需单独启动机械臂
ros2 launch simulated_chassis_ur5e bridge.launch.py rviz:=true

# 如需加载已有地图，用下面命令替代上一条 bridge 命令；定位就绪后启动托管导航
source ~/.bashrc   # 或直接 export SIM_WS_ROOT=/home/kisdy
ros2 launch simulated_chassis_ur5e bridge.launch.py \
  slam_algorithm:=lightning \
  maps_dir:=/home/kisdy/maps \
  pbstream_file:=/home/kisdy/maps/map1025L
```

无界面 GPU 雷达仍需渲染上下文，可用 `xvfb-run -a ros2 launch simulated_chassis_ur5e simulation.launch.py`；Docker 无界面入口已使用 xvfb。

bridge 入口支持 `agv_id`、`port`、`address`、`maps_dir`、`pbstream_file`、`manage_navigation` 等原有配置。`rviz` 默认 false，`allow_fault_injection` 默认 false。启动 bridge 前会检查实际 robot_description、全部控制器和机械臂关节状态就绪；不匹配或重复启动运行节点时拒绝启动机械臂。bridge 重启不会重新生成模型或执行收臂动作。

整机入口只保留机械臂 RViz（`rviz_mobile_ur5e`），托管导航显式关闭自己的 RViz。该窗口同时包含整机模型、MoveIt、地图和导航路径显示；定位后可将 Fixed Frame 改为 `map` 查看全局地图。

在 bridge 所在终端按 Ctrl+C，会同步退出其启动的定位、建图、导航、MoveIt、arm_manager 和 RViz。等待终端返回命令提示符后再重启；退出时先发送中断信号，超时再逐级终止进程组。核心 bridge 节点异常退出也会触发外层 launch 关闭。独立终端启动的仿真仍由仿真终端管理，关闭 RViz 窗口本身不会关闭整套 bridge。

## 配置归属

| 文件 | 用途 |
|---|---|
| `urdf/robot.urdf.xacro` | 底盘宏 + 上装宏 |
| `urdf/arm_mount.xacro` | 安装座惯量/碰撞与官方 UR5e 宏调用 |
| `config/mounting.yaml` | 安装座半径、高度、质量、相对车体位置 |
| `simulated_ur5e/config/initial_positions.yaml` | 单臂、整机、SRDF 共用的折叠初始角度 |
| `config/controllers.yaml` | 仅新增机械臂控制配置，启动时与原底盘参数合并 |
| `config/moveit/robot.srdf.xacro` | 带车体/雷达/安装座的组合碰撞模型和 stowed 位姿 |

安装座默认从车体顶面 z=0.09 m 起，高 0.22 m，质量 3 kg。修改安装配置后，仿真和 bridge 均须传同一 `mounting_file`。初始姿态直接由仿真硬件初始化，不发送开机运动；这是仿真运输姿态，不代表厂商统一出厂姿态。

底盘和机械臂的两个 ros2_control 硬件组件由同一个 Ignition 插件、一个 manager 管理。只保留一个完整 joint_state_broadcaster、robot_state_publisher 和 /clock。MoveIt 只规划机械臂六轴，底盘关节为 passive；车体和雷达保留碰撞体。第一阶段不支持边行走边作业，操作机械臂时保持底盘停车。

原纯底盘的雷达只有 visual，组合 profile 单独启用与其外形一致的圆柱/球体碰撞几何，供机械臂避碰；纯底盘默认不启用，展开模型保持兼容。

## 接口及对比入口

机械臂仍使用 `/arm_manager/command`、`/arm_manager/status` 等原有业务接口，关节名称变为 `ur5e_*`，末端为 `ur5e_tool0`，规划组为 `ur5e_ur_manipulator`。组合示教点单独保存在 `~/.ros/arm_manager/mobile_ur5e_teach_points.json`，不覆盖单臂示教点。

已有 `/joint_states` 和 TF 可观察实际姿态；新的上位机姿态查询接口本阶段未增加。bridge 已允许订阅 manager 状态和调用示教/取消/软件复位服务；模拟故障服务未开放给远端。

纯底盘仍可用 `ros2 launch simulated_chassis three_wheel_sim.launch.py`，独立 UR5e 仍可用 `ros2 launch simulated_ur5e ur5e.launch.py mode:=mock`。它们与整机不能在同一个 ROS_DOMAIN_ID 同时启动。

## 验证

```bash
# 实际 Fortress：折叠初始值、控制器唯一性、整机碰撞、前进/横移/转向、机械臂执行、bridge 重启保持姿态
python3 src/simulated_chassis_ur5e/test/integration_fortress.py
# 附加 3D 建图、保存、定位和托管导航回归
python3 src/simulated_chassis_ur5e/test/integration_fortress.py --navigation
# 增加 RViz 唯一性、正常退出清理、bridge 节点异常退出清理检查（需要 xvfb-run）
python3 src/simulated_chassis_ur5e/test/integration_fortress.py --navigation --shutdown-check
# 服务等待中退出，以及启动器提前退出后遗留子进程的回收检查
python3 src/agv_bridge_v2/scripts/test_managed_shutdown.py
```

测试使用独立 ROS_DOMAIN_ID=92、IGN_PARTITION、临时地图与示教目录，不修改用户地图或示教点。Docker Compose 和入口脚本另做静态检查；镜像仍需按部署流程重建。
