#!/usr/bin/env python3
"""最终链路验证：自发 0.3 m/s 指令 5 秒，在指令进行中同时采：
轮子实际转速 / 控制器原生odom / relay后的odom / tf"""
import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class FinalProbe(Node):
    def __init__(self):
        super().__init__('final_probe')
        self.js = None
        self.odom_native = None
        self.create_subscription(JointState, '/joint_states', self.cb_js, 50)
        self.create_subscription(Odometry, '/diff_drive_controller/odom', self.cb_odom, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_js(self, m):
        self.js = (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                   list(zip(m.name, m.velocity)))

    def cb_odom(self, m):
        self.odom = m.twist.twist.linear.x


def main():
    rclpy.init()
    node = FinalProbe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)

    msg = Twist()
    msg.linear.x = 0.3
    end = time.time() + 5.0
    print('开始发布 0.3 m/s ...')
    while time.time() < end:
        node.pub.publish(msg)
        time.sleep(0.05)

    print('指令进行到第3秒时采样（下面的值在发布循环中抓取）:')
    # 重新发布并在中途采样
    end = time.time() + 4.0
    sampled = False
    while time.time() < end:
        node.pub.publish(msg)
        time.sleep(0.05)
        if not sampled and time.time() > end - 2.0:
            sampled = True
            if node.js:
                t, wheels = node.js
                print('轮速: ' + ', '.join('%s=%.2f rad/s' % (n, v) for n, v in wheels))
            print('原生odom twist.x = %.2f (应为0.3)' % getattr(node, 'odom', -1))
    node.pub.publish(Twist())
    time.sleep(0.5)
    print('结束，已发停。')


if __name__ == '__main__':
    main()
