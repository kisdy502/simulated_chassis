### jzt_robot

AGV 差速底盘仿真包，双激光雷达 + IMU，适配 ROS2 Jazzy + Gazebo Garden/Ionic。

### 编译+启动仿真

```bash
colcon build --packages-select jzt_robot --symlink-install
source install/setup.bash
ros2 launch jzt_robot gazebo_diff_2lidar.launch.py
```

### 二、在线建图

```bash
source install/setup.bash
ros2 launch jzt_robot slam_online_2lidar.launch.py
```

**保存地图：**

```bash
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState \
    "{filename: 'my_map_m.pbstream'}"
```

### 一、录取数据包（用于离线建图）

```bash
ros2 bag record -o my_bag /scan_1 /scan_2 /odom /imu /tf /tf_static /clock
```

### 三、离线建图（离线模式全速处理 bag 数据，不受实时限制，地图精度更高：）

```bash
source install/setup.bash
ros2 launch jzt_robot slam_offline_2lidar.launch.py \
    bag_filenames:=my_bag \
    save_state_filename:=my_map_m_optimized.pbstream
```

### 四、导航

```bash
# 终端2: 启动导航 + Cartographer 纯定位 + RViz
source install/setup.bash
ros2 launch jzt_robot navigation_2lidar.launch.py \
    pbstream_file:=/mnt/d/github/simulated_chassis/my_map_m_optimized.pbstream 
```

---

## 文件结构

```
jzt_robot/
├── launch/
│   ├── gazebo_diff_2lidar.launch.py      # Gazebo 仿真
│   ├── slam_online_2lidar.launch.py       # 在线建图
│   ├── slam_offline_2lidar.launch.py      # 离线建图（bag → pbstream）
│   └── navigation_2lidar.launch.py        # Nav2 导航
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
