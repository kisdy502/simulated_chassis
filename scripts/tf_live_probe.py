#!/usr/bin/env python3
"""实时 /tf 采样：map->odom 的帧率、发布者GID无法直接看，但能看
每秒帧数（正常应=cartographer发布率，约50Hz）与值域抖动。"""
import threading
import time

import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
import math


class TfProbe(Node):
    def __init__(self):
        super().__init__('tf_probe')
        self.mo = []  # (wall, x, y, yaw)
        self.ob = 0
        self.create_subscription(TFMessage, '/tf', self.cb, 500)

    def cb(self, m):
        for tr in m.transforms:
            if tr.header.frame_id == 'map' and tr.child_frame_id == 'odom':
                t = time.time()
                yaw = math.atan2(2*(tr.transform.rotation.w*tr.transform.rotation.z),
                                 1-2*tr.transform.rotation.z**2)
                self.mo.append((t, tr.transform.translation.x,
                                tr.transform.translation.y, yaw))
            elif tr.header.frame_id == 'odom':
                self.ob += 1


def main():
    rclpy.init()
    node = TfProbe()
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    t0 = time.time()
    DUR = 10
    while time.time() < t0 + DUR:
        time.sleep(1.0)
    rows = node.mo
    print('10秒内 map->odom 帧数: %d (率 %.0f Hz)   odom->*帧数: %d' % (
        len(rows), len(rows)/DUR, node.ob))
    if rows:
        xs = [r[1] for r in rows]; ys = [r[2] for r in rows]; yaws = [r[3] for r in rows]
        print('map->odom 值域: x [%.3f, %.3f]  y [%.3f, %.3f]  yaw [%.3f, %.3f]' % (
            min(xs), max(xs), min(ys), max(ys), min(yaws), max(yaws)))
        span = max(max(xs)-min(xs), max(ys)-min(ys))
        print('值域跨度: %.3f m  %s' % (span,
              '→ 正常（单发布者稳定）' if span < 0.1 else '→ 异常！值在大幅翻转（多发布者或剧烈重锚）'))
        # 高频交替检测: 相邻帧间跳变次数
        j = 0
        for i in range(1, len(rows)):
            if math.hypot(rows[i][1]-rows[i-1][1], rows[i][2]-rows[i-1][2]) > 0.05:
                j += 1
        print('>5cm 相邻跳变: %d 次' % j)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
