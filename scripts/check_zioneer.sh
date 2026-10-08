#!/bin/bash
# zioneer 排障：进程/发布者/节点 一览
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== gazebo / 控制相关进程 ==='
ps aux | grep -E 'gz sim|ign gazebo|controller_manager|diff_drive|robot_state' | grep -v grep | awk '{printf "%s %s %s %s\n", $2, $11, $12, $13}'
echo
echo '=== 关键话题发布者数 ==='
for t in /odom /joint_states /clock /tf /cmd_vel /diff_drive_controller/cmd_vel; do
  n=$(timeout 4 ros2 topic info "$t" 2>/dev/null | grep 'Publisher count' | grep -o '[0-9]*')
  echo "$t : $n"
done
echo
echo '=== controller_manager / 硬件插件节点 ==='
timeout 5 ros2 node list 2>/dev/null | grep -E 'controller_manager|ros2_control'
