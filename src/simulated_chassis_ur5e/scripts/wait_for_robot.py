#!/usr/bin/env python3
"""Bounded readiness check, using wall clock even if simulation is paused."""
import re
import time
import xml.etree.ElementTree as ET
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def geometry(xml):
    # mesh URI 归一化:Gazebo 版描述用 file://绝对路径,robot_state_publisher 版用
    # package://(上位机 mesh 重写需要),两者仅此差异不代表模型不同,比对前统一
    xml = re.sub(r'file://[^"\']*?/share/([\w.-]+)/', r'pkg://\1/', xml)
    xml = xml.replace('package://', 'pkg://')
    root = ET.fromstring(xml)
    for tag in ('gazebo', 'ros2_control'):
        for element in list(root.findall(tag)):
            root.remove(element)
    return ET.canonicalize(ET.tostring(root, encoding='unicode'), strip_text=True)


def main():
    rclpy.init()
    node = Node('wait_for_mobile_ur5e')
    expected = geometry(node.declare_parameter('robot_description', '').value)
    timeout = node.declare_parameter('timeout_sec', 90.0).value
    joint_time, matching, error = 0.0, False, ''

    def description(message):
        nonlocal matching, error
        try:
            matching = geometry(message.data) == expected
            if not matching:
                error = 'Running robot model differs from this profile/mounting configuration'
        except Exception as exception:
            error = str(exception)

    def joints(message):
        nonlocal joint_time
        required = {'ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
                    'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint'}
        if required.issubset(message.name):
            joint_time = time.monotonic()

    node.create_subscription(String, '/robot_description', description,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(JointState, '/joint_states', joints, qos_profile_sensor_data)
    client = node.create_client(ListControllers, '/controller_manager/list_controllers')
    deadline, future, last_query = time.monotonic() + timeout, None, 0.0
    active, last_report = False, 0.0
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() - last_report > 5.0:
                node.get_logger().info(f'Waiting: model={matching}, controllers={active}, joint_age={time.monotonic()-joint_time:.1f}s')
                last_report = time.monotonic()
            if error:
                raise RuntimeError(error)
            if future is not None and future.done():
                active = {'three_wheel_base_controller', 'joint_state_broadcaster', 'joint_trajectory_controller'}.issubset(
                    {item.name for item in future.result().controller if item.state == 'active'})
                future = None
            if future is not None and time.monotonic() - last_query > 3.0:
                future.cancel()
                future = None
            if future is None and client.service_is_ready() and time.monotonic() - last_query > 0.5:
                future = client.call_async(ListControllers.Request())
                last_query = time.monotonic()
            if matching and active and time.monotonic() - joint_time < 2.0:
                if any(name in ('move_group', 'arm_manager') for name, namespace in node.get_node_names_and_namespaces()):
                    raise RuntimeError('MoveIt/arm_manager already running; do not start a second arm runtime')
                node.get_logger().info('Combined model and all controllers ready')
                return
        raise RuntimeError('Timed out: start simulation.launch.py and verify /clock and controllers')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
