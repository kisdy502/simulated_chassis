#!/usr/bin/env python3
"""C++ service/action integration against a fake JTC; no Gazebo or robot motion."""
import math
import os
import subprocess
import tempfile
import threading
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '187'
import rclpy
from rclpy.action import ActionServer, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from ament_index_python.packages import get_package_prefix
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    driver = Node('fake_gripper_controller')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(driver)
    group = ReentrantCallbackGroup()
    data = {'publish': False, 'invalid': False, 'reject': False, 'abort': False,
            'goals': [], 'status': '', 'width': None}

    def execute(handle):
        abort = data['abort']
        data['goals'].append(handle.request)
        time.sleep(0.7)  # Keep a real in-flight action to test busy handling.
        result = FollowJointTrajectory.Result()
        if abort:
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            handle.abort()
        else:
            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
            handle.succeed()
        return result

    server = ActionServer(driver, FollowJointTrajectory,
        '/gripper_controller/follow_joint_trajectory', execute_callback=execute,
        goal_callback=lambda _: GoalResponse.REJECT if data['reject'] else GoalResponse.ACCEPT,
        callback_group=group)
    publisher = driver.create_publisher(JointState, '/joint_states', 10)

    def publish():
        if not data['publish']:
            return
        message = JointState()
        message.name = ['ur5e_gripper_left_joint', 'ur5e_gripper_right_joint']
        message.position = [math.nan, 0.03] if data['invalid'] else [0.03, 0.03]
        publisher.publish(message)

    timer = driver.create_timer(0.05, publish, callback_group=group)
    driver.create_subscription(Float64, '/gripper/width', lambda m: data.update(width=m.data), 10)
    driver.create_subscription(String, '/gripper/status', lambda m: data.update(status=m.data),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    client = driver.create_client(SetBool, '/gripper/set_closed', callback_group=group)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    def until(predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError(f'Timed out; state={data}')

    def call(closed):
        future = client.call_async(SetBool.Request(data=closed))
        until(future.done)
        return future.result()

    executable = Path(get_package_prefix('simulated_chassis_ur5e')) / 'lib/simulated_chassis_ur5e/gripper_control'
    with tempfile.TemporaryFile(mode='w+') as log:
        process = subprocess.Popen([str(executable), '--ros-args', '-p', 'command_duration:=0.2'],
            stdout=log, stderr=subprocess.STDOUT)
        try:
            until(client.service_is_ready)
            assert not call(True).success, 'Must reject commands without measured joints'
            data['publish'] = True
            until(lambda: data['width'] is not None)
            assert abs(data['width'] - 0.06) < 1e-6, 'Width must come from measured state'
            time.sleep(0.3)
            assert call(False).success
            until(lambda: len(data['goals']) == 1)
            assert not call(True).success, 'Concurrent service commands must be rejected'
            goal = data['goals'][-1]
            assert list(goal.trajectory.points[0].positions) == [0.03, 0.03]
            assert list(goal.trajectory.points[-1].positions) == [0.04, 0.04]
            until(lambda: data['status'] == 'succeeded')

            assert call(True).success
            until(lambda: len(data['goals']) == 2)
            goal = data['goals'][-1]
            assert list(goal.trajectory.points[-1].positions) == [0.0, 0.0]
            end = goal.trajectory.points[-1].time_from_start
            assert end.sec + end.nanosec * 1e-9 >= 0.75, 'Respect configured finger speed'
            until(lambda: data['status'] == 'succeeded')

            data['reject'] = True
            assert call(False).success  # Service acceptance is not controller acceptance.
            until(lambda: data['status'] == 'rejected')
            data['reject'] = False
            data['abort'] = True
            assert call(False).success
            until(lambda: data['status'] == 'aborted')
            data['invalid'] = True
            until(lambda: data['status'] == 'state_stale')
            assert not call(True).success, 'NaN feedback must not refresh state readiness'
            print('PASS: measured width, open/close targets, speed, busy, rejection, abort, stale/NaN state')
        finally:
            process.terminate()
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait()
            if process.returncode not in (0, -15):
                log.seek(0)
                print(log.read())
            executor.shutdown()
            thread.join(timeout=5)
            server.destroy()
            driver.destroy_timer(timer)
            driver.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
