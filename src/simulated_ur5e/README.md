# UR5e 独立仿真与原型对比

ROS 2 Humble / Ubuntu 22.04 / Ignition Fortress。复用官方 `ur_description` 的 UR5e 模型与 `ur_moveit_config`，不修改 `simulated_arm` 或底盘包。尚未集成夹爪、感知与底盘。

## 安装与编译

在 Linux 的当前工作空间根目录执行（需要 ROS apt 软件源）：

如果只运行 `mode:=mock`，可先安装以下依赖，无需下载 Fortress 官方仿真仓库：

```bash
sudo apt update
sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config \
  ros-humble-moveit ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to simulated_ur5e
source install/setup.bash
ros2 launch simulated_ur5e ur5e.launch.py mode:=mock
```

`package 'ur_description' not found` 表示官方模型依赖尚未安装。仅编译本包不会自动安装系统依赖。完整 Fortress 模式按下面步骤安装：

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install ros-humble-ur-description ros-humble-ur-moveit-config \
  ros-humble-moveit ros-humble-ign-ros2-control ros-humble-ros-gz-sim \
  ros-humble-ros-gz-bridge ros-humble-joint-trajectory-controller \
  ros-humble-joint-state-broadcaster python3-rosdep

# 官方 Fortress 仿真包采用 Humble 分支；不要使用默认 ros2 分支。
git clone -b humble https://github.com/UniversalRobots/Universal_Robots_ROS2_GZ_Simulation.git src/ur_simulation_gz
rosdep install --from-paths src/simulated_ur5e src/ur_simulation_gz --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --packages-up-to simulated_ur5e
source install/setup.bash
```

如果该目录已经存在，复用其 Humble checkout，不要再次 clone。首次使用 rosdep 的系统需先运行 `sudo rosdep init`、`rosdep update`。

核对接口时使用的官方 Humble 仿真提交为 `e49336eb369a3e75fd31753512d4afb3c0c1eb6f`。官方依赖仍由 apt / rosdep 安装，未拷贝厂商模型到本仓库。

## 启动

```bash
# RViz + 模拟硬件：验证模型、IK、规划与轨迹执行，不启动物理仿真
ros2 launch simulated_ur5e ur5e.launch.py mode:=mock

# Fortress + MoveIt2 + RViz（默认模式）
ros2 launch simulated_ur5e ur5e.launch.py

# 无界面
ros2 launch simulated_ur5e ur5e.launch.py gui:=false rviz:=false
```

启动默认包含 `arm_manager` 业务接口。新增包后编译使用：

```bash
colcon build --symlink-install --packages-select arm_manager_interfaces arm_manager simulated_ur5e
source install/setup.bash
```

接口使用方法见 `src/arm_manager/README.md`。可用 `include_arm_manager:=false` 关闭，或 `allow_fault_injection:=true` 开启模拟故障测试。RViz Execute 和业务接口不要同时下发运动任务。

在 RViz MotionPlanning 面板选择 `ur_manipulator`，拖动末端交互标记，先 Plan 再 Execute。建议最初将速度、加速度缩放设为 0.1。使用官方默认初始关节姿态。

控制链路：RViz → MoveIt2 / OMPL → `/joint_trajectory_controller/follow_joint_trajectory` → 标准 JTC → 模拟硬件或 `ign_ros2_control`。

## 与 simulated_arm 对比

先关闭一个仿真，再启动另一个；两个包默认使用全局 `/robot_description`、`/joint_states`、`/controller_manager`、`/move_group` 与 `/clock`，不能直接同域并行启动。需要并行时分别设置不同 `ROS_DOMAIN_ID`，各自的 RViz 与诊断终端也必须使用对应 ID。

| 项目 | simulated_arm | simulated_ur5e |
|---|---|---|
| 模型 | 自定义六轴原型 | 官方 UR5e 模型、网格与参数 |
| RViz 启动 | `arm_rviz_demo.launch.py` | `ur5e.launch.py mode:=mock` |
| Fortress 启动 | `arm_gazebo.launch.py` | `ur5e.launch.py` |
| 规划组 | 以原型 SRDF 为准 | `ur_manipulator` |
| 轨迹接口 | `/arm_controller/follow_joint_trajectory` | `/joint_trajectory_controller/follow_joint_trajectory` |
| 末端坐标系 | `arm_tool0` | `tool0` |

对比时使用相同目标位置与姿态、相同缩放系数，记录可达性、规划成功率、规划耗时、执行耗时与末端误差。不同机械臂的关节角含义不同，不要直接复制六个关节角来判断效果。

```bash
ros2 control list_controllers
ros2 action list -t
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link tool0
```

验收：两个控制器均 active；轨迹 action 存在；Plan & Execute 成功且 Gazebo 同步运动；静止时无持续漂移。Mock 模式只验证控制链路，不验证动力学或接触抓取。

## 后续底盘集成

此包目前用官方独立模型固定到 world。集成时在组合描述中调用官方 `ur_macro.xacro`，挂载到车体安装座，合并底盘与机械臂控制配置，并重新生成含车体、雷达和安装座的 SRDF 碰撞配置；不要直接给此独立模型额外添加底盘。

现有 Docker 镜像只 COPY、编译底盘相关包。要在该镜像中运行本包，需补充以上依赖及新包和官方仿真包的 COPY / colcon 构建步骤。
