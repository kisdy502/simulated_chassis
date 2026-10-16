# 三舵轮 3D 雷达 + UR5e 整机仿真

本包只负责仿真组合：安装座、带 `ur5e_` 前缀的机械臂、组合碰撞/控制配置、启动编排。底盘算法继续在 `simulated_chassis`，机械臂业务在 `arm_manager`，单臂调试在 `simulated_ur5e`，通用/真机 bridge 保持独立。

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arm_manager_interfaces arm_manager simulated_chassis simulated_ur5e simulated_chassis_ur5e
source install/setup.bash

# 终端一：一个世界、一台组合机器人、一个 controller_manager
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true

# 终端二：原有 bridge + MoveIt + arm_manager，无需单独启动机械臂
ros2 launch simulated_chassis_ur5e bridge.launch.py rviz:=true
```

无界面 GPU 雷达仍需渲染上下文，可用 `xvfb-run -a ros2 launch simulated_chassis_ur5e simulation.launch.py`；Docker 无界面入口已使用 xvfb。

bridge 入口支持 `agv_id`、`port`、`address`、`maps_dir`、`pbstream_file`、`manage_navigation` 等原有配置。`rviz` 默认 false，`allow_fault_injection` 默认 false。启动 bridge 前会检查实际 robot_description、全部控制器和机械臂关节状态就绪；不匹配或重复启动运行节点时拒绝启动机械臂。bridge 重启不会重新生成模型或执行收臂动作。

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
```

测试使用独立 ROS_DOMAIN_ID=92、IGN_PARTITION、临时地图与示教目录，不修改用户地图或示教点。Docker Compose 和入口脚本另做静态检查；镜像仍需按部署流程重建。
