#!/usr/bin/env python3
"""Fortress optical-frame normalization and bounded, fresh-frame snapshots."""
import json
import shutil
import tempfile
import threading
import time
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger


class WristCamera(Node):
    def __init__(self):
        super().__init__('wrist_camera')
        self.output_dir = Path(self.declare_parameter(
            'output_dir', '~/.ros/wrist_camera/photos').value).expanduser().resolve()
        self.timeout = float(self.declare_parameter('capture_timeout_sec', 5.0).value)
        if not 0.0 < self.timeout <= 60.0:
            raise ValueError('capture_timeout_sec must be in (0, 60]')
        self.frame = 'ur5e_camera_optical_frame'
        self.cv_bridge = CvBridge()
        self.condition = threading.Condition()
        self.latest = None
        self.info = None
        self.sequence = 0
        self.image_pub = self.create_publisher(Image, '/wrist_camera/image_raw', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/wrist_camera/camera_info', qos_profile_sensor_data)
        self.create_subscription(Image, '/wrist_camera/raw/image', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/wrist_camera/raw/camera_info', self.on_info, qos_profile_sensor_data)
        # A separate group lets sensor callbacks run while the service waits.
        # Serializing service callbacks bounds memory and avoids capture storms.
        self.capture_group = MutuallyExclusiveCallbackGroup()
        self.create_service(Trigger, '/wrist_camera/take_photo', self.capture,
                            callback_group=self.capture_group)

    def on_image(self, message):
        # Fortress may publish a scoped sensor name as frame_id. Pixels remain
        # unchanged; this TF frame matches the sensor pose and ROS optical axes.
        message.header.frame_id = self.frame
        self.image_pub.publish(message)
        with self.condition:
            self.latest = message
            self.sequence += 1
            self.condition.notify_all()

    def on_info(self, message):
        message.header.frame_id = self.frame
        self.info_pub.publish(message)
        with self.condition:
            self.info = message

    def capture(self, request, response):
        requested_stamp = self.get_clock().now().nanoseconds
        deadline = time.monotonic() + self.timeout
        with self.condition:
            sequence = self.sequence
            while True:
                image = self.latest
                if image is not None:
                    stamp = image.header.stamp
                    fresh = stamp.sec * 10**9 + stamp.nanosec > requested_stamp
                    if self.sequence > sequence and fresh:
                        info = self.info
                        break
                remaining = deadline - time.monotonic()
                if remaining <= 0 or not rclpy.ok():
                    response.success = False
                    response.message = 'No new camera frame before timeout; check simulation, rendering and bridge'
                    return response
                self.condition.wait(min(remaining, 0.1))
        folder = None
        try:
            pixels = self.cv_bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            self.output_dir.mkdir(parents=True, exist_ok=True)
            folder = Path(tempfile.mkdtemp(prefix=f'photo_{stamp.sec}_{stamp.nanosec:09d}_',
                                           dir=str(self.output_dir)))
            filename = folder / 'image.png'
            if not cv2.imwrite(str(filename), pixels):
                raise RuntimeError('PNG writer failed')
            metadata = {'stamp': {'sec': stamp.sec, 'nanosec': stamp.nanosec},
                        'frame_id': image.header.frame_id,
                        'width': image.width, 'height': image.height,
                        'camera_info': None}
            # Intrinsics are static for this simulated camera; record their own
            # stamp explicitly, rather than claiming image/info synchronization.
            if info is not None and (info.width, info.height) == (image.width, image.height):
                metadata['camera_info'] = {
                    'stamp': {'sec': info.header.stamp.sec, 'nanosec': info.header.stamp.nanosec},
                    'distortion_model': info.distortion_model,
                    'd': list(info.d), 'k': list(info.k), 'r': list(info.r), 'p': list(info.p)}
            (folder / 'metadata.json').write_text(json.dumps(metadata, indent=2), encoding='utf-8')
            response.success = True
            response.message = str(filename)
        except Exception as error:
            if folder is not None:
                shutil.rmtree(folder, ignore_errors=True)
            response.success = False
            response.message = f'Photo save failed: {error}'
        return response


def main():
    rclpy.init()
    node = WristCamera()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
