# jzt_robot

AGV 机器人仿真包，支持 Gazebo 建图和导航。

## 启动顺序

### 1. 启动 Gazebo 差速仿真（始终运行）

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot gazebo_diff.launch.py
```

### 1.1 启动 Gazebo 阿克曼仿真（始终运行）差速，阿克曼二选一

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot gazebo_ackermann.launch.py \
world_name:=jzt_factory_sz.world
```

> **重要**: Gazebo 启动后保持运行，不要关闭。无论建图还是导航都基于此仿真环境。
> **重要**: 阿克曼的底盘，移动控制话题和差速不一样，用的是
> **重要**: ros2_control (ackermann_steering_controller) /ackermann_steering_controller/reference_unstamped
> **重要**: ros2 topic pub /ackermann_steering_controller/reference_unstamped geometry_msgs/msg/Twist '{linear: {x: 0.6}, angular: {z: 0.4}}' --rate 5

### 1.2 启动 Gazebo 麦克纳姆轮仿真

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot gazebo_mecanum.launch.py use_sim_time:=true
```

---

### 2. 建图模式

````bash
# 在新的终端中启动建图 + RViz
conda deactivate
source install/setup.bash
ros2 launch jzt_robot slam.launch.py

## 双雷达
```bash
ros2 launch slam.launch_double_lidar.py
````

# 键盘控制机器人移动（第三个终端）

ros2 run teleop_twist_keyboard teleop_twist_keyboard

````

**建图完成后保存地图:**

```bash
ros2 service call /write_state \
    cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/kisdy/maps/jzt_factory_map.pbstream'}"
````

> 建图完成后，关闭建图终端（Ctrl+C），再启动导航模式。

---

### 3. 导航模式

<!-- cspell: ignore pbstream kisdy -->

```bash
# 先确保 gazebo 正在运行，然后启动导航 + RViz
conda deactivate
source install/setup.bash
ros2 launch jzt_robot navigation.launch.py \
    params_file:=/home/kisdy/projects/agv_localization_ws/install/jzt_robot/share/jzt_robot/param/nav2_params_mppi_cartographer_ackermann.yaml \
    pbstream_file:=/home/kisdy/maps/jzt_factory_map.pbstream \
    cmd_topic:=/ackermann_steering_controller/reference_unstamped \
    use_sim_time:=true
```

## /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml

## 双雷达导航启动

```bash
conda deactivate
source install/setup.bash
ros2 launch jzt_robot navigation.launch_double_lidar.py \
    params_file:=/home/kisdy/projects/agv_localization_ws/install/jzt_robot/share/jzt_robot/param/nav2_params_mppi_cartographer_ackermann_double_lidar.yaml \
    pbstream_file:=/home/kisdy/maps/jzt_factory_map.pbstream \
    cmd_topic:=/ackermann_steering_controller/reference_unstamped \
    use_sim_time:=true
```

## 标定测试

```bash
# 圆形测试（半径 2.0m，留足余量）
ros2 run jzt_robot ackermann_calib_trajectory_node --ros-args \
  -p mode:=circle \
  -p linear_velocity:=0.5 \
  -p radius:=2.0 \
  -p num_cycles:=3

# 或者 8 字测试（半径 1.5m，接近极限但安全）
ros2 run jzt_robot ackermann_calib_trajectory_node --ros-args \
  -p mode:=figure8 \
  -p linear_velocity:=0.5 \
  -p radius:=1.8 \
  -p num_cycles:=4

```

---

## Launch 文件说明

| 文件                    | 功能                                 |
| ----------------------- | ------------------------------------ |
| `gazebo_diff.launch.py` | Gazebo 仿真环境（需始终运行）        |
| `slam.launch.py`        | Cartographer 建图 + RViz 可视化      |
| `navigation.launch.py`  | Nav2 导航 + Cartographer 定位 + RViz |

## 文件结构

```
jzt_robot/
├── launch/
│   ├── gazebo_diff.launch.py              # Gazebo 仿真
│   ├── slam.launch.py                # 建图模式
│   ├── navigation.launch.py          # 导航模式
├── config/                          # 机器人配置
├── param/                           # Nav2 参数
├── rviz/                            # RViz 配置文件
├── urdf/                            # 机器人模型
└── world/                           # Gazebo 世界文件
```

## 编译

```bash
colcon build --packages-select jzt_robot
```

## jzt_robot 问题大全 vs 解决步骤

1.建图时候，每次重启建图，机器人在地图位置发生偏移了，gazebo和Cartographer重复发布odom到tf坐标变化了2.建图雷达不贴合地图边缘，精度不够，修改差速urdf，将分辨率提高到720，起始结束角度和插件保持一致，降低雷达的噪声3.启动nav2 导航launch报错 原因，我现在用Cartographer建图，Cartographer导航，不能用bring_up.launch.py会默认启动amcl
需改成 navigation_launch.py
4.nav2启动rviz看不到机器人位置，ros2 run tf2_tools view_frames，发现没有map到odom的坐标变化，原因gazebo和Cartographer分别发布了map-odom，base_footprint-odom坐标变化，导致冲突

## 检查imu 的frame id是否正确,检查其他的topic用法也是一样的

```
ros2 topic echo /imu --once | grep frame_id
  frame_id: imu_link
```

## 检查nav2 配置是否生效,例如 bt_xml_filename参数

```
ros2 param get /bt_navigator bt_xml_filename
```

## 修复了问题

```
1启动导航节点，地图不现实，日志提示odom到base_footprint坐标不存在，底盘urdf配置问题，imu的frame id需要设置成imu_link
    localization_2d.lua参数问题，参考官方backup_2d.lua配置，补齐参数
1导航节点启动时候，rviz中雷达轮廓和地图边缘不贴合，
  差速底盘urdf配置有问题，lidar的frame id设置成laser_scan
  localization_2d.lua参数问题，参考官方backpack_2d_localization.lua，补齐参数
```

### 阿克曼底盘，启动失败，自定义行为树配置没有生效，默认行为树的名字没有写对

```
ros2 param get /bt_navigator bt_xml_filename
String value is: /home/kisdy/projects/agv_localization_ws/install/jzt_robot/share/jzt_robot/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml

 [lifecycle_manager_navigation]: Server behavior_server connected with bond.
[lifecycle_manager-10] [INFO] [1777517839.764069996] [lifecycle_manager_navigation]: Activating bt_navigator
[bt_navigator-7] [INFO] [1777517839.764369349] [bt_navigator]: Activating
[bt_navigator-7] [ERROR] [1777517839.764780083] [bt_navigator]: Exception when loading BT: Error at line 30: -> Node not recognized: Spin
[bt_navigator-7] [ERROR] [1777517839.764874781] [bt_navigator]: Error loading XML file: /opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_through_poses_w_replanning_and_recovery.xml
[lifecycle_manager-10] [ERROR] [1777517839.765286296] [lifecycle_manager_navigation]: Failed to change state for node: bt_navigator
[lifecycle_manager-10] [ERROR] [1777517839.765360887] [lifecycle_manager_navigation]: Failed to bring up all requested nodes. Aborting bringup.
```

### 时间戳冲突，某个节点没有被杀掉，产生多个实例，kill掉，重新启动

```
[cartographer_node-1] F0430 15:18:04.755954 2336678 map_by_time.h:43] Check failed: data.time > std::prev(trajectory.end())->first (621355977382090000 vs. 621355977382090000)
[cartographer_node-1] [FATAL] [1777533484.756424053] [cartographer logger]: F0430 15:18:04.000000 2336678 map_by_time.h:43] Check failed: data.time > std::prev(trajectory.end())->first (621355977382090000 vs. 621355977382090000)
```

## 阿克曼底盘到点失败 nav2 阿克曼参数需要优化，少了旋转，到点可能不容易保证角度对准

```
[controller_server-3] [INFO] [1777536465.748209676] [controller_server]: Optimizer reset
[controller_server-3] [INFO] [1777536465.753084792] [controller_server]: Optimizer reset
[controller_server-3] [ERROR] [1777536465.753176116] [controller_server]: Optimizer fail to compute path
[controller_server-3] [WARN] [1777536465.753286516] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[controller_server-3] [INFO] [1777536465.789516848] [controller_server]: Received a goal, begin computing control effort.
[controller_server-3] [INFO] [1777536465.999128055] [controller_server]: Optimizer reset
[controller_server-3] [INFO] [1777536466.004376783] [controller_server]: Optimizer reset
[controller_server-3] [ERROR] [1777536466.004461674] [controller_server]: Optimizer fail to compute path
[controller_server-3] [WARN] [1777536466.004556725] [controller_server]: [follow_path] [ActionServer] Aborting handle.
[bt_navigator-7] [WARN] [1777536466.488983604] [bt_navigator]: [navigate_to_pose] [ActionServer] Aborting handle.
[bt_navigator-7] [ERROR] [1777536466.489215556] [bt_navigator]: Goal failed
```

# 双雷达仿真遇到的坑

```
[cartographer_node-1] [WARN] [1778319965.766929041] [cartographer logger]: W0509 17:46:05.000000 348905 ordered_multi_queue.cc:155] Queue waiting for data: (1, scan_2)




  # Cartographer 定位节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
            # '-start_trajectory_with_default_topics', 'false',
            '--ros-args',
            '--log-level', 'WARN',          # 只显示 ERROR 和 FATAL
        ],
        remappings=[
            ('scan_1', '/scan_front'),      # 第一个雷达：/scan → /scan_front
            ('scan_2', '/scan_rear'),     # 第二个雷达：/scan_1 → /scan_rear
            ('odom', '/odom'),
            ('imu', '/imu'),  # 添加这行！
        ],
    )

    原因定位节点 需要将两个雷达的话题转发过去，但是我写成了,导致cartographer迟迟收不到第二个激光雷达，无法完成定位初始化，导致map-odom的坐标变化无法发布
    remappings=[
            ('scan', '/scan_front'),      # 第一个雷达：/scan → /scan_front
            ('scan_1', '/scan_rear'),     # 第二个雷达：/scan_1 → /scan_rear
            ('odom', '/odom'),
            ('imu', '/imu'),  # 添加这行！
        ],
```

## 优化nav2参数，规避障碍物

## 阿克曼到点配置优化，不然一直无法到达目标点

# 将 xacro文件转成 urdf文件

xacro src/jzt_robot/urdf/mecanum/robot.xacro > /tmp/test.urdf
xacro src/jzt_robot/urdf/ackermann/jz_ackermann_robot.urdf.xacro > /tmp/test.urdf

## 发送移动控制命令

```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 5
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: -0.5, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --rate 1

```

## 查看消息结构

```
ros2 interface show nav_msgs/msg/Odometry
ros2 interface package nav_msgs
ros2 interface show geometry_msgs/msg/PoseWithCovariance
```

## 查看topic

```
# 查看所有 topic
ros2 topic list

# 查看 topic 类型
ros2 topic type /scan

# 查看 topic 结构
ros2 interface show sensor_msgs/msg/LaserScan

# 实时打印 topic 数据
ros2 topic echo /scan

# 查看 topic 发布频率
ros2 topic hz /scan

# 查看 topic 带宽
ros2 topic bw /scan
```

# 查看节点参数

ros2 param list /cartographer_node

# 获取参数值

ros2 param get /cartographer_node map_frame

# 设置参数

ros2 param set /cartographer_node use_sim_time true

# 查看所有 service

ros2 service list

# 查看 service 类型

ros2 service type /service_name

# 手动调用 service

## 自定义nav2 插件

DOCKING_DEBUG: dist=0.2791m x=0.2789 y=-0.0098 yaw=0.0547(3.1°) | cmd_v=0.285 cmd_w=-0.032
[component_container-3] [INFO] [1779442292.586209808] [docking_controller]: DOCKING_DEBUG: dist=0.1023m x=0.1022 y=-0.0037 yaw=0.0522(3.0°) | cmd_v=0.100 cmd_w=-0.019
[component_container-3] [INFO] [1779442293.146908635] [docking_controller]: Docking succeeded! Errors: x=0.0481, y=-0.0011, yaw=0.0682
[component_container-3] [INFO] [1779442293.166290975] [docking_controller]: Docking done, switched back to nav2
[cmd_vel_mux_node-15] [INFO] [1779442293.166412844] [cmd_vel_mux_node]: Switching cmd_vel source: docking -> nav2
[cmd_vel_mux_node-15] [WARN] [1779442293.170374109] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442294.171917418] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[bt_navigator-8] [INFO] [1779442294.247193591] [bt_navigator_navigate_to_pose_rclcpp_node]: DockingAction succeeded
[bt_navigator-8] [INFO] [1779442294.347270144] [bt_navigator]: Goal succeeded
[cmd_vel_mux_node-15] [WARN] [1779442295.180391576] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442296.180398160] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442297.180404323] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442298.180451664] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442299.180479739] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442300.190420800] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
[cmd_vel_mux_node-15] [WARN] [1779442301.190440639] [cmd_vel_mux_node]: Nav2 cmd_vel timeout! Sending zero velocity.
