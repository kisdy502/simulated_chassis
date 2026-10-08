#!/usr/bin/env python3
"""看点云最近回波：机器人周围有没有贴脸障碍（判断卡死 vs 悬空）。"""
import struct
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class MinRange(Node):
    def __init__(self):
        super().__init__('min_range_probe')
        self.got = {}
        for topic in ('/points2_1', '/points2_2'):
            self.create_subscription(PointCloud2, topic,
                                     lambda m, t=topic: self.cb(m, t), 10)

    def cb(self, m, tag):
        step = m.point_step
        best = None
        n = m.row_step * m.height
        data = m.data
        for off in range(0, min(len(data), step * 20000), step):
            x, y, z = struct.unpack_from('fff', data, off)
            r = (x * x + y * y + z * z) ** 0.5
            if r > 0.05 and (best is None or r < best):
                best = r
        self.got[tag] = best

    @staticmethod
    def sector(m):
        return None


def main():
    rclpy.init()
    node = MinRange()
    import time
    end = time.time() + 4
    import threading
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    while time.time() < end and len(node.got) < 2:
        time.sleep(0.2)
    for tag, r in node.got.items():
        print('%s 最近回波: %.2f m' % (tag, r or -1))
    if not node.got:
        print('未收到点云')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
