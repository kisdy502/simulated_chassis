#!/usr/bin/env python3
"""Fault-injection tests for delayed acknowledgment and unresponsive cancellation.

Uses deliberately slow fake MoveIt servers; does not control a real robot.
"""
import importlib.util
import os
import tempfile
import threading
import time
from pathlib import Path

os.environ['ROS_DOMAIN_ID'] = '91'
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from arm_manager_interfaces.action import ArmCommand
from arm_manager_interfaces.srv import InjectFault, SaveTeachPoint, ListTeachPoints
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint
from std_srvs.srv import Trigger


def run():
    with tempfile.TemporaryDirectory(prefix='arm-stop-test-') as directory:
        rclpy.init(args=['--ros-args', '-p', 'simulation_mode:=true',
            '-p', 'allow_fault_injection:=true', '-p', 'stop_timeout:=0.15',
            '-p', f'teach_points_file:={directory}/points.json'])
        spec = importlib.util.spec_from_file_location('manager', Path(__file__).parents[1] / 'scripts/arm_manager_node.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        manager = module.ArmManager()
        fake = Node('fake_moveit')
        group = ReentrantCallbackGroup()
        delay = {'ack': 0.0, 'execution': 0.0, 'executed': 0, 'canceled': 0}

        def accept(request):
            time.sleep(delay['ack'])
            return GoalResponse.ACCEPT

        def plan(handle):
            time.sleep(0.02)
            response = MoveGroup.Result()
            if handle.is_cancel_requested:
                delay['canceled'] += 1
                response.error_code.val = -7
                handle.canceled()
            else:
                response.error_code.val = 1
                response.planned_trajectory.joint_trajectory.joint_names = manager.names
                response.planned_trajectory.joint_trajectory.points = [JointTrajectoryPoint(positions=[0.0] * 6)]
                handle.succeed()
            return response

        def execute(handle):
            delay['executed'] += 1
            time.sleep(delay['execution'])
            response = ExecuteTrajectory.Result()
            response.error_code.val = 1
            handle.succeed()  # intentionally ignores cancellation / stop events
            return response

        planner = ActionServer(fake, MoveGroup, '/move_action', plan, goal_callback=accept,
            cancel_callback=lambda handle: CancelResponse.ACCEPT, callback_group=group)
        execution = ActionServer(fake, ExecuteTrajectory, '/execute_trajectory', execute,
            cancel_callback=lambda handle: CancelResponse.REJECT, callback_group=group)
        joints = fake.create_publisher(JointState, '/joint_states', 10)
        fake.create_timer(0.02, lambda: joints.publish(JointState(name=manager.names, position=[0.0] * 6)),
                          callback_group=group)
        client_node = Node('stop_test_client')
        client = ActionClient(client_node, ArmCommand, '/arm_manager/command')
        pool = MultiThreadedExecutor(num_threads=10)
        for node in (manager, fake, client_node):
            pool.add_node(node)
        thread = threading.Thread(target=pool.spin, daemon=True)
        thread.start()

        def until(predicate, timeout=5):
            end = time.monotonic() + timeout
            while time.monotonic() < end:
                if predicate():
                    return
                time.sleep(0.01)
            raise AssertionError('Condition timed out')

        def wait(future):
            until(future.done)
            return future.result()

        def service(kind, name, request):
            connection = client_node.create_client(kind, '/arm_manager/' + name)
            assert connection.wait_for_service(timeout_sec=3)
            try:
                return wait(connection.call_async(request))
            finally:
                client_node.destroy_client(connection)

        count = 0
        def send(timeout=3):
            nonlocal count
            count += 1
            return wait(client.send_goal_async(ArmCommand.Goal(command_id=f'stop-{count}',
                operation='joint', joint_names=manager.names, joint_positions=[0.0] * 6,
                velocity_scale=0.2, acceleration_scale=0.2, timeout_sec=float(timeout))))

        try:
            assert client.wait_for_server(timeout_sec=3)
            until(manager.ready)
            delay['ack'] = 0.6
            handle = send(0.03)
            outcome = wait(handle.get_result_async()).result
            assert outcome.error_code == 'STOP_UNCONFIRMED', outcome
            assert manager.fault and manager.pending
            assert not service(Trigger, 'reset_fault', Trigger.Request()).success
            assert not send().accepted
            until(lambda: not manager.pending)
            assert delay['executed'] == 0 and delay['canceled'] == 1
            assert service(Trigger, 'reset_fault', Trigger.Request()).success
            print('PASS late goal acknowledgment canceled; lock cannot be reset before terminal result', flush=True)

            delay['ack'], delay['execution'] = 0.0, 0.8
            handle = send()
            until(lambda: manager.state == 'executing' and delay['executed'] == 1)
            assert service(InjectFault, 'inject_fault', InjectFault.Request(reason='fault_during_motion')).success
            outcome = wait(handle.get_result_async()).result
            assert outcome.error_code == 'STOP_UNCONFIRMED', outcome
            assert manager.pending and manager.fault
            assert not service(Trigger, 'reset_fault', Trigger.Request()).success
            until(lambda: not manager.pending)
            assert service(Trigger, 'reset_fault', Trigger.Request()).success
            print('PASS unresponsive execution remains locked until backend terminal result', flush=True)

            assert service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='restart_point')).success
            # Stop callback workers before destroying/replacing a node.
            pool.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            manager.destroy_node()
            manager = module.ArmManager()
            pool = MultiThreadedExecutor(num_threads=10)
            pool.add_node(fake)
            pool.add_node(client_node)
            pool.add_node(manager)
            thread = threading.Thread(target=pool.spin, daemon=True)
            thread.start()
            until(manager.ready)
            points = service(ListTeachPoints, 'list_teach_points', ListTeachPoints.Request()).points
            assert len(points) == 1 and points[0].name == 'restart_point'
            assert list(points[0].joint_positions) == [0.0] * 6
            print('PASS teach points survive node restart', flush=True)

            manager.cfg['simulation_mode'] = False
            assert not service(InjectFault, 'inject_fault', InjectFault.Request(reason='disabled')).success
            assert not service(Trigger, 'reset_fault', Trigger.Request()).success
            manager.cfg['simulation_mode'] = True
            manager.joint_time = 0.0
            # Disable the fake publisher to test missing feedback deterministically.
            for timer in list(fake.timers):
                timer.cancel()
            time.sleep(0.1)
            manager.joint_time = 0.0
            outcome = wait(send().get_result_async()).result
            assert outcome.error_code == 'NOT_READY', outcome
            assert not service(SaveTeachPoint, 'save_teach_point', SaveTeachPoint.Request(name='stale')).success
            print('PASS real-mode recovery disabled and stale feedback rejects motion/teaching', flush=True)
            print('ALL STOP SEMANTICS CHECKS PASSED', flush=True)
        finally:
            time.sleep(0.2)  # drain result/cancel callback tasks before destroying handles
            pool.shutdown(timeout_sec=5)
            thread.join(timeout=5)
            planner.destroy()
            execution.destroy()
            client.destroy()
            for node in (manager, fake, client_node):
                node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    run()
