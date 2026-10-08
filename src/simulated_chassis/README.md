## 三舵轮机器人实现（3D雷达）

colcon build --packages-select simulated_chassis --symlink-install
source install/setup.bash
ros2 launch simulated_chassis three_wheel_sim.launch.py

# 新八边形世界
ros2 launch simulated_chassis three_wheel_sim.launch.py world:=world_octagon.sdf


## 2D 雷达版（当前默认机器人）

本包默认机器人 = 2D 雷达版三舵轮（three_wheel_chassis_2d.xacro）：复刻真机
"底盘角镂空 + 雷达低位角装"，左前/右后斜对角各一个 2D 雷达（水平 270°/540 点/
15Hz/量程 0.10~25m），扫描面离地 0.105m，朝外 45°/-135°，双雷达 FOV 并集全向
360°，盲区朝内（轮组/对角雷达互不可见，无自打点）。
真实机器人出厂即固定一种雷达，仿真同样把所有 launch 默认值固定为 2D——
上层（bridge/前端）零参数，启动即 2D 机器人在跑。

# 1. 仿真（默认加载 2D 机器人 xacro）
ros2 launch simulated_chassis three_wheel_sim.launch.py

# 2. 导航（默认 2D 定位 lua；nav2 消费 /scan_1 /scan_2）
ros2 launch simulated_chassis navigation.launch.py include_localization:=false

# 3. bridge（托管定位/建图自动用 2D 配置，无需额外参数）
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    robot_package:=simulated_chassis pbstream_file:=<2D版地图>.pbstream

# 在线建图（默认 slam_2d_lidar_online.lua；上位机 /agv/start_mapping 同效）
ros2 launch simulated_chassis slam.launch.py
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map_2d.pbstream'}"

# 录包（2D 版录 LaserScan，不录 points2）
ros2 bag record -o my_bag_2d /scan_1 /scan_2 /odom /imu /tf /tf_static /clock

# 离线建图（默认 slam_2d_lidar_offline.lua + 2D 机器人 xacro）
ros2 launch simulated_chassis slam_offline.launch.py \
    bag_filenames:=my_bag_2d save_state_filename:=my_map_2d.pbstream

# 导航/定位（不走 bridge 时同样默认 2D 配置）
ros2 launch simulated_chassis navigation.launch.py pbstream_file:=/path/to/my_map_2d.pbstream

## 切换 3D 仿真（改 launch 文件，不常做）

仿真机器人与离线建图的 xacro 已固定为 2D 版，需要 3D 时手动改文件：
1. three_wheel_sim.launch.py：xacro 行改为 three_wheel_chassis_3d.xacro，
   并按文件内注释把 bridge 参数集换成 3D 版（补 PointCloud2 桥接）
2. slam_offline.launch.py：xacro 行改为 three_wheel_chassis_3d.xacro
3. 建图/导航配置仍可命令行传参：
ros2 launch simulated_chassis slam.launch.py configuration_basename:=slam_3d_online.lua
ros2 launch simulated_chassis slam_offline.launch.py \
    configuration_basename:=slam_3d_offline.lua bag_filenames:=<bag> save_state_filename:=<pbstream>
ros2 launch simulated_chassis navigation.launch.py \
    configuration_basename:=localization_3d.lua pbstream_file:=<3D版地图>.pbstream

bridge 托管链路默认 2D：AgvNavServerNode 的 /start_trajectory 默认 lua 已改为
localization_2d_lidar.lua（与 simulated_chassis 各 launch 默认一致）。
3D 会话请走手动 navigation.launch.py 方式，不要经 bridge 托管定位。

注意：2D 扫描面（0.105m）与 3D（0.25m）特征高度不同，两版地图不可混用。

## 在线建图
source install/setup.bash
ros2 launch simulated_chassis slam.launch.py

# 保存为 .pbstream（Cartographer 原生格式）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map.pbstream'}"

# 转换为 .pgm + .yaml
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename my_map_m_optimized.pbstream \
    -map_filestem my_map_m_optimized

# 先录制 bag 包（在线建图时录制）
ros2 bag record -o my_bag --topics /points2 /odom /imu /tf /tf_static /clock

# 正确录制方式（只录原始数据）
ros2 bag record -o my_bag \
    /points2 \
    /odom \
    /imu  \
    /tf_static \
    /clock

# 离线建图
ros2 launch simulated_chassis slam_offline.launch.py \
    bag_filenames:=my_bag \
    save_state_filename:=my_map_optimized.pbstream


# 离线建图 (有问题，优化后的地图体积很小，不正常，正在研究如何解决)
cd ~/workspace/simulated_chassis

source install/setup.bash
ros2 launch simulated_chassis slam_offline.launch.py \
    bag_filenames:="/home/kisdy/workspace/simulated_chassis/my_bag" \
    save_state_filename:="/home/kisdy/workspace/simulated_chassis/my_map_optimized.pbstream"


# 启动导航
# 导航 launch 已拆分：localization.launch.py（Cartographer 3D 纯定位，可独立重启）
# + navigation.launch.py（Nav2，默认仍包含定位，独立使用行为不变）
source install/setup.bash
ros2 launch simulated_chassis navigation.launch.py \
    pbstream_file:=/mnt/d/github/simulated_chassis/my_map_optimized.pbstream

# 上位机地图管理模式（定位/建图由 agv_nav_server 托管，navigation 只启动 Nav2；
# 支持 /agv/load_map 切图、/agv/start_mapping 建图、/agv/save_map 保存）：
ros2 launch simulated_chassis navigation.launch.py include_localization:=false
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    pbstream_file:=$PWD/maps/my_map_optimized/my_map_optimized.pbstream \
    robot_package:=simulated_chassis


## 前进
ros2 topic pub /three_wheel_base_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: -0.2, y: 0.0}, angular: {z: 0.0}}' --rate 2

## 原地旋转
ros2 topic pub /three_wheel_base_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.5}}' --rate 2


## 平移
ros2 topic pub /three_wheel_base_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: -0.5}, angular: {z: 0.0}}' --rate 2

## 停止
ros2 topic pub /three_wheel_base_controller/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0}, angular: {z: 0.0}}' --rate 1


## gazebo效果图
![alt text](images/image.png)

## 在线建图rviz 预览效果
![alt text](images/image2.png)

## 离线建图地图导航效果图
![alt text](images/image4.png)

## 子图显示
![alt text](images/image3.png)

## gazebo

# 查看当前 Gazebo 版本
ign gazebo --version

ign topic -e -t /clock

# 控制器状态查看

## slam 建图遇到几个坑
```
1，tf完整，但是rviz没有地图，仿真时候，需要指定imu和雷达的frame_id <ignition_frame_id>lidar_link</ignition_frame_id> ,<ignition_frame_id>imu_link</ignition_frame_id>
2，步骤1做了，但是还是没地图，建图时候，gazebo修改世界，将机器人模型保存到了世界中，导致slam建图，提示雷达坐标系不存在，urdf目录加载的机器人被世界的机器人覆盖了，frame id异常了
3、slam建图和离线建图，配置目前都用保守参数，
4、nav2导航，配置参数雷达话题要和实际话题一致，不然无法显示地图
5、离线建图，有bug，还在解决中（已经解决，离线建图，关闭仿真和在线建图等）
6、三舵轮控制器需要优化，现在还有很多问题，舵轮最大旋转角度，没有做限制，转到目标角度时候，需要归一化，按最小转角旋转，rviz观察有漂移现象，还在定位问题
7，话题处理，需要转成points2话题，不然后面离线建图只录包话题只能用points2，
8、三舵轮控制器，目前取消了发布tf和odom，因为gazebo插件在发布，后面可以自己发布，验证tf和odom计算是否准确
9、在线和离线建图参数调整，不能和源码差距太离谱，比如离线建图参数
   -- 些尝试性优化，看能不能大幅度提升建图质量
    --  每个节点都优化（离线不受实时限制）
    POSE_GRAPH.optimize_every_n_nodes = 30 
    -- ✅ 最终优化：大量迭代打磨结果
    POSE_GRAPH.max_num_final_iterations = 300 之前用1000，离线建图直接闪退
```
