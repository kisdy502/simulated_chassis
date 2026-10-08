#!/bin/bash
# 重定位后导航 TF 断链排查
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== 1. 仿真时钟活着吗(3秒) ==='
timeout 4 ros2 topic hz /clock 2>&1 | tail -1

echo
echo '=== 2. 节点: 定位链在不在 ==='
timeout 5 ros2 node list 2>/dev/null | grep -E 'cartographer|tracked_pose'

echo
echo '=== 3. /tracked_pose 频率(carto在发吗) ==='
timeout 4 ros2 topic hz /tracked_pose 2>&1 | tail -1

echo
echo '=== 4. map->odom 最新时间戳 vs 当前仿真时间 ==='
python3 - <<'EOF'
import time, threading, math
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

rclpy.init()
n = Node('tf_age_probe')
st = {'clock': None, 'mo': None, 'ob': None}

def cb_clock(m):
    st['clock'] = m.clock.sec + m.clock.nanosec*1e-9

def cb_tf(msg):
    for tr in msg.transforms:
        if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
            st['mo'] = tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9
        elif tr.header.frame_id == 'odom' and tr.child_frame_id == 'base_footprint':
            st['ob'] = tr.header.stamp.sec + tr.header.stamp.nanosec*1e-9

n.create_subscription(Clock, '/clock', cb_clock, 50)
n.create_subscription(TFMessage, '/tf', cb_tf, 500)
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
time.sleep(3)
now = st['clock'] or 0
def fmt(x):
    return '无数据' if x is None else '%.3f (距now %+.3fs)' % (x, x - now)
print('当前仿真时间 now = %.3f' % now)
print('map->odom 最新戳:', fmt(st['mo']))
print('odom->base 最新戳:', fmt(st['ob']))
rclpy.shutdown()
EOF
