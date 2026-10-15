#!/usr/bin/env python3
"""Real MoveIt/UR5e mock integration. Run with workspace sourced; isolated DDS domain.

python3 src/arm_manager/test/integration_mock.py
"""
import json
import os
import signal
import subprocess
import tempfile
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '89'
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from arm_manager_interfaces.action import ArmCommand
from arm_manager_interfaces.msg import ArmStatus
from arm_manager_interfaces.srv import SaveTeachPoint, ListTeachPoints, DeleteTeachPoint, InjectFault
from moveit_msgs.srv import GetPositionFK, ApplyPlanningScene
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger


def run():
    with tempfile.TemporaryDirectory(prefix='arm-manager-test-') as directory:
        log = open(Path(directory) / 'launch.log', 'w+')
        storage = str(Path(directory) / 'points.json')
        process = subprocess.Popen(['ros2', 'launch', 'simulated_ur5e', 'ur5e.launch.py',
            'mode:=mock', 'rviz:=false', 'allow_fault_injection:=true',
            f'teach_points_file:={storage}'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init()
        node = Node('arm_manager_integration_test')
        statuses = []
        node.create_subscription(ArmStatus, '/arm_manager/status', statuses.append,
            QoSProfile(depth=20, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        action = ActionClient(node, ArmCommand, '/arm_manager/command')

        def until(predicate, timeout=20):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.03)
                if predicate():
                    return
                if process.poll() is not None:
                    raise RuntimeError('Launch exited unexpectedly')
            raise AssertionError('Timed out waiting for condition')

        def future_result(future, timeout=30):
            until(future.done, timeout)
            return future.result()

        def service(kind, name, request):
            client = node.create_client(kind, '/arm_manager/' + name)
            assert client.wait_for_service(timeout_sec=5), name
            try:
                return future_result(client.call_async(request))
            finally:
                node.destroy_client(client)

        sequence = 0
        def command(operation='joint', positions=None, **kwargs):
            nonlocal sequence
            sequence += 1
            goal = ArmCommand.Goal(command_id=f'test-{sequence}', operation=operation,
                velocity_scale=0.2, acceleration_scale=0.2, timeout_sec=30.0)
            if positions is not None:
                goal.joint_names = list(statuses[-1].joint_names)
                goal.joint_positions = list(positions)
            for key, value in kwargs.items():
                setattr(goal, key, value)
            return goal

        def send(goal):
            return future_result(action.send_goal_async(goal))

        def result(handle):
            assert handle.accepted
            value = future_result(handle.get_result_async(), 45).result
            until(lambda: statuses and not statuses[-1].busy)
            return value

        try:
            assert action.wait_for_server(timeout_sec=20)
            until(lambda: statuses and statuses[-1].ready)
            initial = list(statuses[-1].joint_positions)
            assert service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='baseline')).success
            assert not service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='baseline')).success
            assert not service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='../bad')).success
            assert service(ListTeachPoints, 'list_teach_points', ListTeachPoints.Request()).points[0].name == 'baseline'
            assert json.loads(Path(storage).read_text())['points']['baseline'] == initial
            print('PASS teaching persistence, list, overwrite/name validation', flush=True)

            target = list(initial)
            target[0] += 0.25
            first = command(positions=target)
            assert result(send(first)).success
            until(lambda: abs(statuses[-1].joint_positions[0] - target[0]) < 0.01)
            assert not send(first).accepted  # ID dedup
            assert result(send(command('named', target_name='baseline'))).success
            until(lambda: abs(statuses[-1].joint_positions[0] - initial[0]) < 0.01)
            print('PASS joint motion, measured target, named recall, duplicate rejection', flush=True)

            fk_client = node.create_client(GetPositionFK, '/compute_fk')
            assert fk_client.wait_for_service(timeout_sec=5)
            fk = GetPositionFK.Request()
            fk.header.frame_id = 'base_link'
            fk.fk_link_names = ['tool0']
            fk.robot_state.joint_state.name = list(statuses[-1].joint_names)
            fk.robot_state.joint_state.position = list(statuses[-1].joint_positions)
            response = future_result(fk_client.call_async(fk))
            assert response.error_code.val == 1
            assert result(send(command('pose', target_pose=response.pose_stamped[0]))).success
            # IK can select another joint branch for the same tool pose.
            assert result(send(command('named', target_name='baseline'))).success
            until(lambda: max(abs(a - b) for a, b in zip(statuses[-1].joint_positions, initial)) < 0.01)
            print('PASS Cartesian pose goal through real IK/MoveIt', flush=True)

            assert result(send(command(positions=initial, velocity_scale=0.0))).error_code == 'INVALID_ARGUMENT'
            assert result(send(command('named', target_name='missing'))).error_code == 'INVALID_ARGUMENT'

            # Add a real PlanningScene obstacle encompassing the starting robot.
            scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene')
            assert scene_client.wait_for_service(timeout_sec=5)
            scene = ApplyPlanningScene.Request()
            scene.scene.is_diff = True
            obstacle = CollisionObject(id='test_blocker')
            obstacle.header.frame_id = 'base_link'
            obstacle.primitives = [SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[4.0, 4.0, 4.0])]
            obstacle.primitive_poses = [Pose()]
            obstacle.primitive_poses[0].orientation.w = 1.0
            obstacle.operation = CollisionObject.ADD
            scene.scene.world.collision_objects = [obstacle]
            assert future_result(scene_client.call_async(scene)).success
            failed = result(send(command(positions=target)))
            assert failed.error_code == 'PLANNING_FAILED', failed
            obstacle.operation = CollisionObject.REMOVE
            assert future_result(scene_client.call_async(scene)).success
            print('PASS invalid commands and collision-blocked planning', flush=True)

            slow_target = list(initial)
            slow_target[0] += 0.3
            slow = send(command(positions=slow_target, velocity_scale=0.01, acceleration_scale=0.01))
            until(lambda: statuses[-1].state == 'executing')
            assert not send(command(positions=initial)).accepted
            assert not service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='moving')).success
            assert future_result(slow.cancel_goal_async()).goals_canceling
            canceled = result(slow)
            assert canceled.error_code == 'CANCELED', canceled
            print('PASS busy exclusion and action cancellation', flush=True)

            slow = send(command(positions=slow_target, velocity_scale=0.01, acceleration_scale=0.01))
            until(lambda: statuses[-1].state == 'executing')
            assert service(Trigger, 'cancel', Trigger.Request()).success
            assert result(slow).error_code == 'CANCELED'
            assert not service(Trigger, 'cancel', Trigger.Request()).success
            print('PASS service cancellation', flush=True)

            short = result(send(command(positions=slow_target, timeout_sec=0.001)))
            assert short.error_code == 'TIMEOUT', short
            print('PASS wall-clock timeout', flush=True)

            slow = send(command(positions=slow_target, velocity_scale=0.01, acceleration_scale=0.01))
            until(lambda: statuses[-1].state == 'executing')
            assert service(InjectFault, 'inject_fault', InjectFault.Request(reason='simulated_collision')).success
            assert result(slow).error_code == 'INJECTED_FAULT'
            until(lambda: statuses[-1].fault_latched)
            assert not send(command(positions=initial)).accepted
            assert service(Trigger, 'reset_fault', Trigger.Request()).success
            until(lambda: statuses[-1].ready)
            assert result(send(command('named', target_name='baseline'))).success
            assert service(DeleteTeachPoint, 'delete_teach_point', DeleteTeachPoint.Request(name='baseline')).success
            assert not service(ListTeachPoints, 'list_teach_points', ListTeachPoints.Request()).points
            print('PASS active fault injection, lockout, reset, recovery and delete', flush=True)
            print('ALL REAL MOVEIT MOCK INTEGRATION CHECKS PASSED', flush=True)
        except Exception:
            log.flush()
            log.seek(0)
            print(log.read()[-18000:], flush=True)
            raise
        finally:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            node.destroy_node()
            rclpy.shutdown()
            log.close()


if __name__ == '__main__':
    run()
