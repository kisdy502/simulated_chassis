### jzt_robot

AGV 差速底盘仿真包，双激光雷达 + IMU，适配 ROS2 Humble + Gazebo Fortress。

### 编译+启动仿真

```bash
colcon build --packages-select jzt_robot --symlink-install
source install/setup.bash
ros2 launch jzt_robot gazebo_diff_2lidar.launch.py
```

### 一、在线建图

```bash
source install/setup.bash
ros2 launch jzt_robot slam.launch.py
```

**保存地图：**

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: 'my_map_m.pbstream'}"
```

### 二、录取数据包（用于离线建图）

```bash
ros2 bag record -o my_bag /scan_1 /scan_2 /odom /imu /tf /tf_static /clock
```

### 三、离线建图（离线模式全速处理 bag 数据，不受实时限制，地图精度更高：）

```bash
source install/setup.bash
ros2 launch jzt_robot slam_offline.launch.py \
    bag_filenames:=my_bag \
    save_state_filename:=my_map_m_optimized.pbstream
```

### 四、导航

导航 launch 已拆分为「定位 + Nav2」两段：`localization.launch.py`（Cartographer
纯定位 + occupancy_grid + pointcloud_to_laserscan）与 Nav2 全栈解耦，
`navigation.launch.py` 默认仍包含定位，独立使用行为不变。

```bash
# 方式一：独立使用（定位 + 导航一体，默认 include_localization:=true）
source install/setup.bash
ros2 launch jzt_robot navigation.launch.py \
    pbstream_file:=/mnt/d/github/simulated_chassis/my_map_m_optimized.pbstream
```

```bash
# 方式二：上位机地图管理模式（定位由 agv_nav_server 托管，navigation 只启动 Nav2）
ros2 launch jzt_robot navigation.launch.py include_localization:=false

ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    pbstream_file:=$PWD/maps/my_map_m_optimized.pbstream
#（agv_bridge_v2 默认 robot_package=jzt_robot，定位/建图 launch 从本包拉起）

# 切图：地图三件套（.pbstream + .pgm + .yaml）放 maps/ 后
ros2 service call /agv/load_map agv_bridge_v2_interfaces/srv/LoadMap "{map_name: 'my_map_2'}"
ros2 topic echo /agv/status   # mode: RELOCALIZING -> NAVIGATION 即切换完成
```

### 五、上位机在线建图（可选，不依赖 RViz 手动保存）

前提同方式二（bridge 托管定位，`navigation.launch.py include_localization:=false`）：

```bash
# 1. 进入建图（自动停定位、拉起 slam.launch.py；mode=MAPPING，导航任务被拒绝，/cmd_vel 遥控可用）
ros2 service call /agv/start_mapping agv_bridge_v2_interfaces/srv/StartMapping "{}"

# 2. 遥控探索建图（或上位机经 rosbridge 发 /cmd_vel）
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' --rate 2

# 3. 保存并回到定位（自动：/write_state 存 pbstream -> 转 pgm/yaml 三件套 -> 停建图 -> 拉定位）
ros2 service call /agv/save_map agv_bridge_v2_interfaces/srv/SaveMap "{map_name: 'my_map_new'}"
ros2 topic echo /agv/status   # mode: MAPPING -> RELOCALIZING -> NAVIGATION 即完成
```

全场景下 Nav2 导航进程与仿真进程均不需要重启，只有定位/建图子进程被切换。

---

## 文件结构

```
jzt_robot/
├── launch/
│   ├── gazebo_diff_2lidar.launch.py      # Gazebo 仿真
│   ├── slam.launch.py                    # 在线建图（bridge 建图模式拉起的单元）
│   ├── slam_offline.launch.py            # 离线建图（bag → pbstream）
│   ├── localization.launch.py            # Cartographer 纯定位（bridge 托管/切图单元）
│   └── navigation.launch.py              # Nav2 导航（include_localization 控制是否含定位）
├── config/
│   ├── slam_2d_online.lua                 # 在线建图 Cartographer 配置
│   ├── slam_2d_offline.lua                # 离线建图 Cartographer 配置
│   ├── localization_2d_double_lidar.lua   # 纯定位 Cartographer 配置
│   ├── diff_controllers.yaml              # ros2_control 差速控制器
│   ├── ekf_localization.yaml              # EKF 融合配置
│   └── gamepad_config.yaml                # 手柄配置
├── param/
│   └── nav2_params_mppi_cartographer*.yaml # Nav2 MPPI 导航参数
├── urdf/diff/
│   ├── robot.xacro                        # 总入口（robot_name 可传参）
│   ├── ros2_control_sim.xacro             # Jazzy ros2_control
│   └── sensors_gazebo.xacro               # 双激光 + IMU 传感器
└── world/
    └── world_sm.sdf                       # 仿真世界
```

## 常用调试命令

```bash
# 话题
ros2 topic list
ros2 topic hz /scan_1
ros2 topic echo /joint_states

# 控制器状态
ros2 control list_controllers
ros2 control list_hardware_interfaces

# 手动发速度测试
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
    "{linear: {x: 0.5}}" --rate 10

# 保存地图
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: '/home/think/maps/my_map.pbstream'}"
```
