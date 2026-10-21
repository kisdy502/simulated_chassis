#!/usr/bin/env python3
"""ROS-only capture regression; no Gazebo or moving robot required."""
import importlib.util
import json
import tempfile
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


def main():
    spec = importlib.util.spec_from_file_location('camera',
        Path(__file__).resolve().parents[1] / 'scripts' / 'wrist_camera_node.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    camera = module.WristCamera()
    camera.timeout = 0.3
    driver = Node('wrist_camera_test')
    publisher = driver.create_publisher(Image, '/wrist_camera/raw/image', qos_profile_sensor_data)
    client = driver.create_client(Trigger, '/wrist_camera/take_photo')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(camera)
    executor.add_node(driver)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    try:
        assert client.wait_for_service(timeout_sec=5)
        deadline = time.monotonic() + 5
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert publisher.get_subscription_count() > 0

        def result(future):
            end = time.monotonic() + 3
            while not future.done() and time.monotonic() < end:
                time.sleep(0.01)
            assert future.done(), 'Service deadlock'
            return future.result()

        with tempfile.TemporaryDirectory() as directory:
            camera.output_dir = Path(directory)
            start = time.monotonic()
            assert not result(client.call_async(Trigger.Request())).success
            assert time.monotonic() - start < 2
            print('PASS no frame returns bounded failure')
            future = client.call_async(Trigger.Request())
            for _ in range(12):
                image = camera.cv_bridge.cv2_to_imgmsg(np.zeros((4, 6, 3), dtype=np.uint8), 'bgr8')
                publisher.publish(image)  # stamp=0: old queued data must fail
                time.sleep(0.03)
            assert not result(future).success
            assert not list(Path(directory).iterdir())
            print('PASS stale frames are rejected without files')
            future = client.call_async(Trigger.Request())
            end = time.monotonic() + 2
            while not future.done() and time.monotonic() < end:
                image.header.stamp = driver.get_clock().now().to_msg()
                publisher.publish(image)
                time.sleep(0.02)
            reply = result(future)
            assert reply.success, reply.message
            assert cv2.imread(reply.message).shape == (4, 6, 3)
            metadata = json.loads(Path(reply.message).with_name('metadata.json').read_text())
            assert metadata['frame_id'] == 'ur5e_camera_optical_frame'
            print('PASS fresh frame saves PNG and optical-frame metadata')
    finally:
        executor.shutdown()
        thread.join()
        driver.destroy_node()
        camera.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
