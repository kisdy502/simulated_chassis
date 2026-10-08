#!/bin/bash
# save_map 失败排查：/map QoS + 手动调用保存服务
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== /map 发布者 QoS ==='
timeout 5 ros2 topic info /map -v 2>/dev/null | grep -E 'Publisher count|Durability|Reliability|Node name' | head -10

echo
echo '=== 手动调用 /map_saver/save_map（30s 硬超时） ==='
timeout 30 ros2 service call /map_saver/save_map nav2_msgs/srv/SaveMap \
  "{map_topic: /map, map_url: /tmp/test_save_map1008, image_format: pgm, map_mode: scale, free_thresh: 0.25, occupied_thresh: 0.65}" 2>&1 | tail -5

echo
echo '=== 产物检查 ==='
ls -la /tmp/test_save_map1008* 2>/dev/null || echo '文件未生成'
