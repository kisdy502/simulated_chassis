#!/usr/bin/env python3
"""临时诊断工具：抓取 /joint_states /odom /tf /cmd_vel 到 CSV，用于分析里程计跳变。

用法（WSL，先 source ROS 与工作区）:
  python3 scripts/capture_diag.py probe     # 自测：0.25m/s 前进 2.5s，自动退出
  python3 scripts/capture_diag.py session   # 长时抓取，Ctrl+C 结束（配合手柄复现问题）

输出: /tmp/cap/joint_states.csv  轮/舵关节原始位置（回绕检测的关键）
      /tmp/cap/odom.csv          控制器里程计（回绕是否已修好的关键）
      /tmp/cap/tf.csv            map->odom 与 odom->base_footprint（区分控制器 vs cartographer）
      /tmp/cap/cmd.csv           /cmd_vel（参照指令）
"""
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import Twist


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


STEER = ('wheel_front_steering_joint', 'wheel_left_steering_joint', 'wheel_right_steering_joint')
WHEEL = ('wheel_front_wheel_joint', 'wheel_left_wheel_joint', 'wheel_right_wheel_joint')


class Capture(Node):
    def __init__(self):
        super().__init__('capture_diag')
        os.makedirs('/tmp/cap', exist_ok=True)
        self.f_js = open('/tmp/cap/joint_states.csv', 'w')
        self.f_js.write('t,front_steer,left_steer,right_steer,front_wheel,left_wheel,right_wheel\n')
        self.f_odom = open('/tmp/cap/odom.csv', 'w')
        self.f_odom.write('t,x,y,yaw,vx,vy,wz\n')
        self.f_tf = open('/tmp/cap/tf.csv', 'w')
        self.f_tf.write('t,parent,child,x,y,yaw\n')
        self.f_cmd = open('/tmp/cap/cmd.csv', 'w')
        self.f_cmd.write('t_wall,vx,vy,wz\n')
        self.create_subscription(JointState, '/joint_states', self.cb_js, 100)
        self.create_subscription(Odometry, '/odom', self.cb_odom, 100)
        self.create_subscription(TFMessage, '/tf', self.cb_tf, 200)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 20)
        self.create_subscription(Twist, '/three_wheel_base_controller/cmd_vel', self.cb_cmd, 20)
        self.pub = self.create_publisher(Twist, '/three_wheel_base_controller/cmd_vel', 10)

    def cb_js(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        d = dict(zip(m.name, m.position))
        row = [t] + [d.get(n, float('nan')) for n in STEER + WHEEL]
        self.f_js.write('%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n' % tuple(row))

    def cb_odom(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.f_odom.write('%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f\n' % (
            t, p.x, p.y, yaw_of(m.pose.pose.orientation),
            m.twist.twist.linear.x, m.twist.twist.linear.y, m.twist.twist.angular.z))

    def cb_tf(self, m):
        for tr in m.transforms:
            pair = (tr.header.frame_id, tr.child_frame_id)
            if pair in (('map', 'odom'), ('odom', 'base_footprint')):
                t = tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9
                self.f_tf.write('%.6f,%s,%s,%.6f,%.6f,%.6f\n' % (
                    t, pair[0], pair[1],
                    tr.transform.translation.x, tr.transform.translation.y,
                    yaw_of(tr.transform.rotation)))

    def cb_cmd(self, m):
        self.f_cmd.write('%.6f,%.4f,%.4f,%.4f\n' % (
            time.time(), m.linear.x, m.linear.y, m.angular.z))

    def flush(self):
        for f in (self.f_js, self.f_odom, self.f_tf, self.f_cmd):
            f.flush()


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'session'
    rclpy.init()
    node = Capture()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()

    if mode == 'probe':
        time.sleep(0.5)
        msg = Twist()
        msg.linear.x = 0.25
        end = time.time() + 2.5
        while time.time() < end:
            node.pub.publish(msg)
            time.sleep(0.05)
        node.pub.publish(Twist())
        time.sleep(0.5)
        print('probe 完成')
    elif mode == 'probe2':
        # 受控实验：静止3s -> 0.3m/s 直行8s(约2.4m) -> 静止6s观察map是否追平
        def drive(v, dur):
            msg = Twist()
            msg.linear.x = v
            end = time.time() + dur
            while time.time() < end:
                node.pub.publish(msg)
                time.sleep(0.05)
            node.pub.publish(Twist())
        print('阶段1: 静止 3s')
        time.sleep(3.0)
        print('阶段2: 0.3 m/s 直行 8s')
        drive(0.3, 8.0)
        print('阶段3: 静止 6s (观察 map->odom 是否继续移动)')
        time.sleep(6.0)
        print('probe2 完成')
    else:
        print('抓取中... 用手柄复现问题后 Ctrl+C 结束')
        try:
            while True:
                time.sleep(1)
                node.flush()
        except KeyboardInterrupt:
            pass

    node.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
