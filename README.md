## 三舵轮机器人实现（3D雷达）

colcon build --packages-select simulated_chassis --symlink-install
source install/setup.bash
ros2 launch simulated_chassis three_wheel_sim.launch.py

## 在线建图
source install/setup.bash
ros2 launch simulated_chassis slam3d_online.launch.py

# 保存为 .pbstream（Cartographer 原生格式）
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: 'my_map.pbstream'}"

# 转换为 .pgm + .yaml
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename my_map.pbstream \
    -map_filestem my_map

# 先录制 bag 包（在线建图时录制）
ros2 bag record -o my_bag /points2 /odom /imu /tf /tf_static /clock

# 正确录制方式（只录原始数据）
ros2 bag record -o my_bag \
    /points2 \
    /odom \
    /imu  \
    /tf_static \
    /clock

# 离线建图
ros2 launch simulated_chassis slam3d_offline.launch.py \
    bag_filenames:=my_bag \
    save_state_filename:=my_map_optimized.pbstream


# 离线建图 (有问题，优化后的地图体积很小，不正常，正在研究如何解决)
cd ~/workspace/simulated_chassis

ros2 launch simulated_chassis slam3d_offline.launch.py \
    bag_filenames:="/home/kisdy/workspace/simulated_chassis/my_bag" \
    save_state_filename:="/home/kisdy/workspace/simulated_chassis/my_map_optimized.pbstream"


# 启动导航
ros2 launch simulated_chassis navigation.launch.py \
    pbstream_file:=/home/kisdy/workspace/simulated_chassis/my_map_optimized.pbstream \
    use_sim_time:=true


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
ign topic -e -t /clock

# 控制器状态查看

## slam 建图遇到几个坑
```
1，tf完整，但是rviz没有地图，仿真时候，需要指定imu和雷达的frame_id <ignition_frame_id>lidar_link</ignition_frame_id> ,<ignition_frame_id>imu_link</ignition_frame_id>
2，步骤1做了，但是还是没地图，建图时候，gazebo修改世界，将机器人模型保存到了世界中，导致slam建图，提示雷达坐标系不存在，urdf目录加载的机器人被世界的机器人覆盖了，frame id异常了
3、slam建图和离线建图，配置目前都用保守参数，
4、nav2导航，配置参数雷达话题要和实际话题一致，不然无法显示地图
5、离线建图，有bug，还在解决中
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
