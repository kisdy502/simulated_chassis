# 仿真机器人+agv_bridge节点

## 启动顺序

### 1仿真机器人

source install/setup.bash
ros2 launch simulated_chassis three_wheel_sim.launch.py

### 2导航

ros2 launch simulated_chassis navigation.launch.py include_localization:=false

### 3桥接节点
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py  robot_package:=simulated_chassis
