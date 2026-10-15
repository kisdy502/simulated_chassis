# simulated_arm 六轴机械臂仿真（研究阶段）

自研基础六轴垂直多关节机械臂（J1 基座回转 + J2 肩 + J3 肘 + J4 前臂回转 + J5 腕俯仰 + J6 法兰回转，零位整臂竖直，全展开约 1.02m），用于打通
**底盘 + 机械臂 + MoveIt2** 技术链路，后续选型真机后仅替换 URDF 与控制器参数。

## 依赖安装（Humble + Ignition Fortress）

```bash
sudo apt install ros-humblemoveit ros-humble-ign-ros2-control ros-humble-ros-ign-gazebo ros-humble-ros-gz-bridge \
                 ros-humble-joint-trajectory-controller ros-humble-joint-state-broadcaster
```

## 编译

```bash
colcon build --packages-select simulated_arm --symlink-install
source install/setup.bash
```

## 方式一：RViz 纯规划演示（不启动 Gazebo）

```bash
ros2 launch simulated_arm arm_rviz_demo.launch.py
```

RViz 中用 MotionPlanning 面板：拖动 `arm_tool0` 处的交互标记设置目标位姿
（或在 Goal State 下拉选 `home` / `extended` / `folded`），点 **Plan & Execute**。

## 方式二：Gazebo + RViz 联动仿真

```bash
ros2 launch simulated_arm arm_gazebo.launch.py
# 无界面模式（docker）
ros2 launch simulated_arm arm_gazebo.launch.py gui:=false rviz:=false
```

- Gazebo 中机械臂锚定在世界原点，位置指令直写关节（理想位置执行，研究阶段无需调 PID）
- RViz 中规划并执行，Gazebo 同步看到关节运动
- 规划链路：RViz → move_group(OMPL) → arm_controller(JTC) → ign_ros2_control → Gazebo

## 命令行直接下发轨迹（不走 RViz）

```bash
ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [arm_joint1, arm_joint2, arm_joint3, arm_joint4, arm_joint5, arm_joint6],
    points: [{positions: [0.5, -0.8, 1.4, 0.0, -0.6, 0.5], time_from_start: {sec: 3}}]}}"
```

## 常用检查

```bash
ros2 control list_controllers          # 应看到 arm_controller(active)
ros2 topic echo /joint_states          # 关节状态
ros2 run tf2_tools view_frames         # world -> arm_base_link -> ... -> arm_tool0
```

## 装配到三舵轮底盘（后续步骤）

底盘 xacro（three_wheel_chassis_2d/3d.xacro）中：

```xml
<xacro:include filename="$(find simulated_arm)/urdf/arm6_macro.xacro"/>
<xacro:arm6_macro parent_link="base_link"
                 mount_xyz="0.15 0 0.12" mount_rpy="0 0 0"
                 use_ignition="true"/>
```

注意：合并后整套机器人只有一个 ros2_control `<gazebo>` 插件块（底盘已有
`ign_ros2_control` 插件时，机械臂的 `controllers_yaml` 需与底盘控制器合并到同一
份配置，xacro 宏的 `controllers_yaml` 参数可指定合并后的文件）。

## 目录结构

```
urdf/
  arm6_macro.xacro            # 机械臂宏（挂载到任意 parent_link）
  arm6_standalone.urdf.xacro  # 独立版（world 锚定）
config/
  ros2_controllers_sim.yaml   # Gazebo 内 controller_manager 配置
  ros2_controllers_fake.yaml  # RViz mock 演示配置
  simulated_arm.srdf          # 规划组/位姿/碰撞对
  kinematics.yaml             # KDL IK
  joint_limits.yaml           # 规划限速/加速度
  ompl_planning.yaml          # OMPL 规划器
  moveit_controllers.yaml     # move_group 轨迹执行
  moveit.rviz                 # RViz 配置
launch/
  arm_rviz_demo.launch.py     # RViz 纯规划
  arm_gazebo.launch.py        # Gazebo + RViz 联动
worlds/
  arm_world.sdf               # 空世界（地面 + 光源）
```

## 说明

- Humble + Ignition Fortress 专用（`ign_ros2_control` / `ros_ign_gazebo`）；迁移到
  Jazzy/Harmonic 时需替换为 `gz_ros2_control` / `ros_gz_sim`，URDF/MoveIt 配置可复用
- 首选初始位姿 `home = [0, -0.45, 0.9, 0, -0.45, 0]`（xacro `initial_value` 与 SRDF
  `home` 一致），避免竖直零位的运动学奇异
