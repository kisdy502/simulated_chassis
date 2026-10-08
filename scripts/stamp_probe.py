#!/usr/bin/env python3
"""同时采样 /clock 与 /diff_drive_controller/cmd_vel 的戳，算实时年龄差。
自己以 20Hz 发 /cmd_vel 0.3，持续 5 秒，打印每条转发指令的 stamp vs 最新 clock。"""
import threading
import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist, TwistStamped


class Probe(Node):
    def __init__(self):
        super().__init__('stamp_probe')
        self.clock_sec = 0.0
        self.create_subscription(Clock, '/clock', self.cb_clock, 50)
        self.create_subscription(TwistStamped, '/diff_drive_controller/cmd_vel',
                                 self.cb_cmd, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def cb_clock(self, m):
        self.clock_sec = m.clock.sec + m.clock.nanosec * 1e-9

    def cb_cmd(self, m):
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        age = self.clock_sec - stamp
        print('指令戳=%.3f  最新clock=%.3f  年龄差=%+.3f s  %s' % (
            stamp, self.clock_sec, age,
            '超时(>0.5)会被控制器丢弃!' if age > 0.5 else '新鲜'))


def main():
    rclpy.init()
    node = Probe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    time.sleep(1.0)
    msg = Twist()
    msg.linear.x = 0.3
    end = time.time() + 5.0
    while time.time() < end:
        node.pub.publish(msg)
        time.sleep(0.05)
    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
