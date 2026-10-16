#!/usr/bin/env python3
"""Actual Fortress+bridge regression, with an isolated DDS/IGN partition and temporary data."""
import os
import math
import sys
import signal
import subprocess
import tempfile
import time
from pathlib import Path
os.environ['ROS_DOMAIN_ID'] = '92'
os.environ['IGN_PARTITION'] = 'mobile-ur5e-integration'
os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, PointCloud2
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from arm_manager_interfaces.msg import ArmStatus
from arm_manager_interfaces.action import ArmCommand
from moveit_msgs.srv import GetStateValidity
from controller_manager_msgs.srv import ListControllers
from agv_bridge_v2_interfaces.srv import StartMapping, SaveMap
from agv_bridge_v2_interfaces.msg import AgvStatus
from simulated_chassis_ur5e.configuration import read_yaml, share


def marked_processes(token):
    result = {}
    marker = ('MOBILE_BRIDGE_TEST=' + token).encode()
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            if marker in (entry / 'environ').read_bytes().split(b'\0'):
                result[int(entry.name)] = (entry / 'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace')
        except OSError:
            pass
    return result


def run():
    navigation = '--navigation' in sys.argv
    shutdown_check = '--shutdown-check' in sys.argv
    with tempfile.TemporaryDirectory(prefix='mobile-ur5e-test-') as directory:
        (Path(directory) / 'maps').mkdir()
        (Path(directory) / 'process').mkdir()
        os.environ['AGV_PROCESS_LOG_DIR'] = str(Path(directory) / 'process')
        processes, logs, tokens = [], [], {}
        def launch(arguments):
            log = open(Path(directory) / f'{len(processes)}.log', 'w+')
            logs.append(log)
            env = os.environ.copy()
            token = None
            if 'bridge.launch.py' in arguments:
                token = f'{Path(directory).name}-{len(logs)}'
                env['MOBILE_BRIDGE_TEST'] = token
                if shutdown_check:
                    arguments = ['xvfb-run', '-a'] + arguments
            proc = subprocess.Popen(arguments, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=env)
            if token: tokens[proc.pid] = token
            processes.append(proc)
            return proc
        launch(['xvfb-run', '-a', 'ros2', 'launch', 'simulated_chassis_ur5e', 'simulation.launch.py'])
        launch(['ros2', 'launch', 'simulated_chassis_ur5e', 'bridge.launch.py', 'port:=19092',
                f'manage_navigation:={str(navigation).lower()}', f'rviz:={str(shutdown_check).lower()}',
                f'maps_dir:={directory}/maps', f'teach_points_file:={directory}/points.json'])
        rclpy.init()
        node = Node('mobile_ur5e_integration')
        data = {'joint': None, 'status': None, 'odom': None, 'points': 0, 'map': None, 'agv': None}
        node.create_subscription(JointState, '/joint_states', lambda msg: data.update(joint=msg), qos_profile_sensor_data)
        node.create_subscription(ArmStatus, '/arm_manager/status', lambda msg: data.update(status=msg), 10)
        node.create_subscription(Odometry, '/odom', lambda msg: data.update(odom=msg), 10)
        node.create_subscription(PointCloud2, '/points2_1', lambda msg: data.update(points=data['points'] + 1), qos_profile_sensor_data)
        node.create_subscription(OccupancyGrid, '/map', lambda msg: data.update(map=msg), 1)
        node.create_subscription(AgvStatus, '/agv/status', lambda msg: data.update(agv=msg), 10)

        def until(condition, timeout=100):
            deadline = time.monotonic() + timeout
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
                if condition():
                    return
                if any(proc.poll() is not None for proc in processes):
                    raise AssertionError('Launch exited')
            raise AssertionError('Timeout; state=' + str(data['status']))

        def wait(future, timeout=30):
            until(future.done, timeout)
            return future.result()

        def service(kind, name, request):
            client = node.create_client(kind, name)
            # NavigationManager initially waits for absent Nav2 action servers;
            # arm readiness alone does not mean bridge services are advertised yet.
            assert client.wait_for_service(timeout_sec=45), name
            response = wait(client.call_async(request))
            node.destroy_client(client)
            return response

        try:
            until(lambda: data['status'] is not None and data['status'].ready and data['points'] > 2)
            initial = read_yaml(os.path.join(share('simulated_ur5e'), 'config', 'initial_positions.yaml'))
            q = dict(zip(data['joint'].name, data['joint'].position))
            assert len(q) == 12, q
            assert max(abs(q['ur5e_' + key] - value) for key, value in initial.items()) < 0.03, q
            controllers = service(ListControllers, '/controller_manager/list_controllers', ListControllers.Request())
            assert len([item for item in controllers.controller if item.state == 'active']) == 3
            names = node.get_node_names_and_namespaces()
            # MoveIt itself creates two graph entries named move_group internally.
            for name in ('controller_manager', 'robot_state_publisher', 'arm_manager'):
                assert sum(item[0] == name for item in names) == 1, names
            print('PASS single combined model/manager, three active controllers, folded initial joints, 3D lidar', flush=True)
            request = GetStateValidity.Request()
            request.group_name = 'ur5e_ur_manipulator'
            request.robot_state.joint_state = data['joint']
            validity = service(GetStateValidity, '/check_state_validity', request)
            assert validity.valid, validity.contacts
            print('PASS stowed posture collision-free in combined MoveIt scene', flush=True)

            if navigation:
                reply = service(StartMapping, '/agv/start_mapping', StartMapping.Request())
                assert reply.success, reply.message
                until(lambda: data['map'] is not None and data['map'].info.width > 0, 180)
                print('PASS bridge starts 3D Cartographer mapping and publishes occupancy map', flush=True)

            pub = node.create_publisher(Twist, '/cmd_vel', 10)
            until(lambda: data['odom'] is not None)
            origin = data['odom'].pose.pose.position
            start_x, start_y = origin.x, origin.y
            forward = Twist(); forward.linear.x = 0.1
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                pub.publish(forward)
                rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Twist())
            until(lambda: abs(data['odom'].twist.twist.linear.x) < 0.01, 15)
            point = data['odom'].pose.pose.position
            assert ((point.x-start_x)**2 + (point.y-start_y)**2)**0.5 > 0.03
            print('PASS bridge-owned velocity relay drives loaded chassis and stops', flush=True)

            start_y = data['odom'].pose.pose.position.y
            lateral = Twist(); lateral.linear.y = 0.08
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                pub.publish(lateral)
                rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Twist())
            until(lambda: abs(data['odom'].twist.twist.linear.y) < 0.01, 15)
            assert abs(data['odom'].pose.pose.position.y - start_y) > 0.02
            def yaw():
                q = data['odom'].pose.pose.orientation
                return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            start_yaw = yaw()
            spin = Twist(); spin.angular.z = 0.15
            deadline = time.monotonic() + 6
            while time.monotonic() < deadline:
                pub.publish(spin)
                rclpy.spin_once(node, timeout_sec=0.05)
            pub.publish(Twist())
            until(lambda: abs(data['odom'].twist.twist.angular.z) < 0.01, 15)
            assert abs(math.atan2(math.sin(yaw()-start_yaw), math.cos(yaw()-start_yaw))) > 0.03
            print('PASS loaded chassis lateral travel and in-place rotation', flush=True)

            client = ActionClient(node, ArmCommand, '/arm_manager/command')
            assert client.wait_for_server(timeout_sec=10)
            goal = ArmCommand.Goal(command_id='mounted-motion', operation='joint',
                joint_names=['ur5e_' + key for key in initial],
                joint_positions=list(initial.values()), velocity_scale=0.1, acceleration_scale=0.1, timeout_sec=40.0)
            goal.joint_positions[0] += 0.15
            handle = wait(client.send_goal_async(goal))
            assert handle.accepted
            result = wait(handle.get_result_async(), 50).result
            assert result.success, result
            print('PASS mounted arm planning/execution through unchanged manager API', flush=True)
            client.destroy()

            if navigation:
                reply = service(SaveMap, '/agv/save_map', SaveMap.Request(map_name='upper_body_test'))
                assert reply.success, reply.message
                until(lambda: data['agv'] is not None and data['agv'].pose_initialized
                      and data['agv'].mode == 'NAVIGATION', 180)
                for suffix in ('pbstream', 'pgm', 'yaml'):
                    assert (Path(directory) / 'maps' / 'upper_body_test' / ('upper_body_test.' + suffix)).exists()
                until(lambda: any(name == 'controller_server' for name, ns in node.get_node_names_and_namespaces()), 90)
                print('PASS save map, localization convergence and managed Nav2 restart', flush=True)

            if shutdown_check:
                commands = marked_processes(tokens[processes[1].pid])
                rviz = [cmd for cmd in commands.values() if '/rviz2 ' in cmd]
                assert len(rviz) == 1 and 'rviz_mobile_ur5e' in rviz[0], rviz
                if navigation:
                    assert any('navigation.launch.py' in cmd and 'use_rviz:=false' in cmd for cmd in commands.values())
                print('PASS exactly one arm RViz; managed navigation explicitly disables its RViz', flush=True)

            # Restart only bridge; actual joint position must not be reinitialized.
            before = dict(zip(data['joint'].name, data['joint'].position))
            old_token = tokens[processes[1].pid]
            os.killpg(processes[1].pid, signal.SIGINT)
            processes[1].wait(timeout=40)
            processes.pop(1)
            until(lambda: not marked_processes(old_token), 10)
            print('PASS bridge SIGINT reaps all marked navigation/localization/arm/RViz descendants', flush=True)
            data['status'] = None
            restart = ['ros2', 'launch', 'simulated_chassis_ur5e', 'bridge.launch.py', 'port:=19092',
                f'manage_navigation:={str(navigation).lower()}', f'maps_dir:={directory}/maps', f'teach_points_file:={directory}/points.json']
            if navigation and shutdown_check:
                restart += [f'pbstream_file:={directory}/maps/upper_body_test/upper_body_test.pbstream', 'rviz:=true']
            launch(restart)
            until(lambda: data['status'] is not None and data['status'].ready)
            after = dict(zip(data['joint'].name, data['joint'].position))
            assert abs(after['ur5e_shoulder_pan_joint'] - before['ur5e_shoulder_pan_joint']) < 0.03
            print('PASS bridge restart preserves actual arm posture', flush=True)
            if shutdown_check and navigation:
                token = tokens[processes[1].pid]
                until(lambda: any('localization.launch.py' in cmd for cmd in marked_processes(token).values()), 60)
                server_pid = next(pid for pid, cmd in marked_processes(token).items() if '/agv_nav_server ' in cmd)
                os.kill(server_pid, signal.SIGKILL)
                processes[1].wait(timeout=40)
                deadline = time.monotonic() + 12
                while marked_processes(token) and time.monotonic() < deadline:
                    time.sleep(.1)
                assert not marked_processes(token), marked_processes(token)
                print('PASS forced bridge-node death also stops nested launch, arm runtime and RViz', flush=True)
            print('ALL FORTRESS COMPOSITION CHECKS PASSED', flush=True)
        except Exception:
            for log in logs:
                log.flush(); log.seek(0)
                print(log.read()[-14000:], flush=True)
            raise
        finally:
            for proc in reversed(processes):
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                if proc.poll() is None:
                    try:
                        proc.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait()
            node.destroy_node()
            rclpy.shutdown()
            for log in logs:
                log.close()
            for token in tokens.values():
                for pid in marked_processes(token):
                    try: os.kill(pid, signal.SIGKILL)
                    except ProcessLookupError: pass


if __name__ == '__main__':
    run()
