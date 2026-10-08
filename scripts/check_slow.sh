#!/bin/bash
# 移动缓慢排查：RTF(仿真时钟比例) + 指令速度 + 实际速度
source /opt/ros/humble/setup.bash
source /mnt/d/github/simulated_chassis/install/setup.bash

echo '=== 1. RTF 测量: /odom 期望100Hz(仿真率), 实测/100 = 仿真时钟比例 ==='
timeout 5 ros2 topic hz /odom 2>&1 | tail -2

echo
echo '=== 2. /clock 走 3 秒墙钟, 看仿真时间涨多少 ==='
python3 - <<'EOF'
import time
import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

rclpy.init()
n = Node('rtf_probe')
state = {'t': None, 'wall': None}
def cb(m):
    if state['t'] is None:
        state['t'] = m.clock.sec + m.clock.nanosec*1e-9
        state['wall'] = time.time()
rclpy.create_node
n.create_subscription(Clock, '/clock', cb, 10)
import threading
threading.Thread(target=rclpy.spin, args=(n,), daemon=True).start()
t0 = state['wall']
while state['t'] is None and time.time()-(t0 or time.time()) < 2:
    time.sleep(0.1)
first_t, first_w = state['t'], state['wall']
time.sleep(3.0)
dt_wall = time.time() - first_w
dt_sim = state['t'] - first_t if state['t'] else 0
print('3秒墙钟内 仿真时间走了 %.2fs -> RTF = %.2f' % (dt_sim, dt_sim/max(dt_wall,0.01)))
rclpy.shutdown()
EOF

echo
echo '=== 3. 当前指令速度(/cmd_vel, 抓2秒) ==='
timeout 3 ros2 topic echo /cmd_vel --once 2>/dev/null | head -8 || echo '  (无指令发布)'

echo
echo '=== 4. 机器人实际速度(/odom twist) ==='
timeout 3 ros2 topic echo /odom --field twist.twist --once 2>/dev/null | head -10

echo
echo '=== 5. CPU 占用 TOP ==='
ps aux --sort=-%cpu | head -8 | awk '{printf "%-6s %-5s %s\n", $2, $3, $11}'
