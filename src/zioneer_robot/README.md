##  差速机器人 + 双3D雷达仿真（Cartographer 3D 建图 + Nav2 导航）
## 融合 `jzt_robot` 差速底盘 与 `simulated_chassis` 双3D雷达方案。

### 编译

```bash
colcon build --packages-select zioneer_robot --symlink-install
source install/setup.bash
```

### 1. 仿真启动

```bash
ros2 launch zioneer_robot gazebo_diff_3dlidar.launch.py
```

- 差速底盘（双轮 + 四万向轮），ros2_control + diff_drive_controller
- 双 Mid-360 仿真3D雷达（前+后），输出 `/points2_1`、`/points2_2`
- IMU `/imu`，里程计 `/odom`
- xterm 键盘遥控窗口

### 2. 在线建图

> 先启动仿真，再开新终端：

```bash
source install/setup.bash
ros2 launch zioneer_robot slam3d_online.launch.py
```

保存地图：

```bash
# 保存 .pbstream
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map.pbstream'}"

# 转换 .pgm + .yaml
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename my_map.pbstream \
    -map_filestem my_map
```

### 3. 离线建图

```bash
# 在线建图时录制 bag（只录原始数据）
ros2 bag record -o my_bag \
    /points2_1 \
    /points2_2 \
    /odom \
    /imu \
    /tf_static \
    /clock

# 离线建图（关闭仿真和在线建图后执行）
ros2 launch zioneer_robot slam3d_offline.launch.py \
    bag_filenames:=/home/kisdy/workspace/simulated_chassis/my_bag \
    save_state_filename:=/home/kisdy/workspace/simulated_chassis/my_map_optimized.pbstream
```

### 4. 导航

```bash
source install/setup.bash
ros2 launch zioneer_robot navigation.launch.py \
    pbstream_file:=/home/kisdy/workspace/simulated_chassis/my_map_optimized.pbstream
```

### 键盘控制（teleop）

```bash
# 前进
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2, y: 0.0}, angular: {z: 0.0}}' --rate 2

# 原地旋转
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}' --rate 2

# 停止
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}' --rate 1
```

### 架构说明

| 功能 | 来源 |
|---|---|
| 差速底盘 URDF / 轮子 / 脚轮 / ros2_control | jzt_robot |
| 双3D雷达（gpu_lidar, Mid-360仿真）+ IMU 噪声模型 | simulated_chassis |
| Cartographer 3D Lua 配置（在线/离线/定位） | simulated_chassis |
| Nav2 参数（MPPI DiffDrive + 双点云代价地图） | simulated_chassis（Omni→DiffDrive） |
| 世界文件 world_m.sdf / world_sm.sdf | jzt_robot |

**话题约定：**
- `/points2_1` 前雷达点云，`/points2_2` 后雷达点云
- `/imu`、`/odom`、`/clock`
- `/cmd_vel` → relay → `/diff_drive_controller/cmd_vel`

**TF树：** `map` → `odom` → `base_footprint` → `base_link` → {wheels, casters, front_lidar_link, rear_lidar_link, imu_link}

### 参数调整要点

- 差速运动学：`config/diff_controllers.yaml`（wheel_separation=0.36, wheel_radius=0.09）
- MPPI 差速控制：`param/nav2_params_3d.yaml`（motion_model=DiffDrive, vx_max=1.0, wz_max=1.0）
- 3D建图参数：`config/slam_3d_online.lua`（num_point_clouds=2, num_accumulated_range_data=2）
