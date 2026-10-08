#!/bin/bash
# 控制器接口占用与原生 odom 检查
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== 硬件接口占用状态 ==='
timeout 6 ros2 control list_hardware_interfaces 2>/dev/null

echo
echo '=== 控制器明细 ==='
timeout 6 ros2 control list_controllers -v 2>/dev/null | head -40

echo
echo '=== 控制器原生 odom（绕过 relay）5 秒采样 ==='
timeout 5 ros2 topic hz /diff_drive_controller/odom 2>&1 | head -3

echo
echo '=== 边发指令边看原生 odom twist ==='
(timeout 4 ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" >/dev/null 2>&1 &)
sleep 1.5
timeout 3 ros2 topic echo /diff_drive_controller/odom --field twist.twist.linear.x --once 2>/dev/null
sleep 3
