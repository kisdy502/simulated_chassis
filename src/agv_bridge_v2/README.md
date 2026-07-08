# 功能简介

提供WebSocket server端，接收仿真agv发过来的控制指令，将agv硬件层的位置，姿态等数据发送给仿真机器人
调用仿真机器人提供的agv 控制接口

# 编译 agv_bridge_v2

colcon build --packages-select agv_bridge_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release

# launch（使用默认参数）

```
source install/setup.bash
ros2 launch agv_bridge_v2 agv_bridge_v2.launch.py
```

# launch（覆盖参数，例如连接远端 SpringBoot）

```
source install/setup.bash
ros2 launch agv_bridge_v2 agv_bridge_v2.launch.py \
    use_sim_time:=false \
    agv_id:=three_wheel_agv \
    springboot_host:=172.30.208.1 \
    springboot_port:=22777 \
    websocket_port:=9090 \
    imu_serial_port:=/dev/ttyACM0 \
    imu_baud_rate:=115200
```

# 方法2: 直接运行节点

```
source install/setup.bash
ros2 run agv_bridge_v2 agv_bridge_node \
  --ros-args \
  -p use_sim_time:=False \
  -p agv_id:=AGV001 \
  -p springboot_host:=127.0.0.1 \
  -p springboot_port:=22777 \
  -p websocket_port:=9090 \
  -p imu_serial_port:=/dev/ttyACM0 \
  -p imu_baud_rate:=115200
```