#!/bin/bash
# 导航"无响应"排查：TF链 / 生命周期 / 目标通道
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== 1. 关键节点在不在 ==='
timeout 5 ros2 node list 2>/dev/null | grep -E 'cartographer|tracked_pose|bt_navigator|controller_server|planner_server|behavior_server|lifecycle' | sort

echo
echo '=== 2. map->odom TF（无响应的头号嫌疑） ==='
timeout 4 ros2 topic echo /tf --once 2>/dev/null | grep -B2 -A1 'child_frame_id: odom' | head -6 || echo '  (无输出)'

echo
echo '=== 3. bt_navigator 生命周期状态 ==='
timeout 4 ros2 lifecycle get /bt_navigator 2>/dev/null
timeout 4 ros2 lifecycle get /controller_server 2>/dev/null
timeout 4 ros2 lifecycle get /planner_server 2>/dev/null

echo
echo '=== 4. 导航 action 服务器 ==='
timeout 5 ros2 action list 2>/dev/null | grep -E 'navigate|follow_path'

echo
echo '=== 5. /map 有没有人发 ==='
timeout 4 ros2 topic info /map 2>/dev/null | grep -E 'Type|Publisher'
