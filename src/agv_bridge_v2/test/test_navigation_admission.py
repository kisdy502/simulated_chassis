#!/usr/bin/env python3
"""Isolated action contract test; fake TF/Nav2 only, no robot commands."""
import os
import signal
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient, ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import TransformStamped
from lifecycle_msgs.srv import GetState
from rcl_interfaces.msg import Log
from nav2_msgs.action import Spin, FollowPath, NavigateToPose
from tf2_ros import TransformBroadcaster
from agv_bridge_v2_interfaces.action import FollowEdge
from agv_bridge_v2_interfaces.msg import AgvStatus
from agv_bridge_v2_interfaces.srv import SetControl


def wait_until(predicate, timeout=12):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(0.02)
    raise AssertionError("Timed out waiting for isolated action test condition")


def value(future):
    wait_until(future.done)
    return future.result()


def main():
    assert os.environ.get("ROS_DOMAIN_ID") == "93", "Requires isolated ROS domain 93"
    rclpy.init()
    node = rclpy.create_node("admission_contract_test")
    group = ReentrantCallbackGroup()
    services = []
    def lifecycle(_req, response):
        response.current_state.id = 3
        return response
    for name in ["controller_server", "smoother_server", "planner_server", "behavior_server",
                 "bt_navigator", "waypoint_follower", "velocity_smoother"]:
        services.append(node.create_service(GetState, name + "/get_state", lifecycle, callback_group=group))
    shutting_down = threading.Event()
    complete_actions = threading.Event()
    abort_cause = [None]
    error_publisher = node.create_publisher(Log, "/rosout", 10)
    completed_paths = []
    spin_targets = []
    def spin_execute(handle):
        spin_targets.append(handle.request.target_yaw)
        while not handle.is_cancel_requested and not shutting_down.is_set() and not complete_actions.is_set():
            feedback = Spin.Feedback()
            feedback.angular_distance_traveled = 0.0
            handle.publish_feedback(feedback)
            time.sleep(0.05)
        if handle.is_cancel_requested:
            handle.canceled()
        elif complete_actions.is_set():
            handle.succeed()
        else:
            handle.abort()
        return Spin.Result()
    servers = [ActionServer(node, Spin, "spin", execute_callback=spin_execute,
        goal_callback=lambda _goal: GoalResponse.ACCEPT,
        cancel_callback=lambda _goal: CancelResponse.ACCEPT, callback_group=group)]
    for action, name in [(FollowPath, "follow_path"), (NavigateToPose, "navigate_to_pose")]:
        def execute(handle, action_type=action):
            if action_type is FollowPath:
                completed_paths.append(handle.request.path)
            cause = abort_cause[0]
            abort_cause[0] = None
            if cause is not None:
                if cause:
                    error = Log()
                    error.name = "controller_server"
                    error.level = 40  # Humble byte constant Log.ERROR is bytes in Python; field requires int.
                    error.stamp = node.get_clock().now().to_msg()
                    error.msg = cause
                    error_publisher.publish(error)
                    time.sleep(0.2)  # Allow the independent diagnostic subscription to receive it.
                handle.abort()
            else:
                handle.succeed()
            return action_type.Result()
        servers.append(ActionServer(node, action, name,
            execute_callback=execute,
            goal_callback=lambda _goal: GoalResponse.ACCEPT if complete_actions.is_set() else GoalResponse.REJECT,
            callback_group=group))
    tf = TransformBroadcaster(node)
    def broadcast():
        transforms = []
        stamp = node.get_clock().now().to_msg()
        for parent, child in [("map", "odom"), ("odom", "base_footprint"), ("base_footprint", "base_link")]:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = parent
            t.child_frame_id = child
            t.transform.rotation.w = 1.0
            transforms.append(t)
        tf.sendTransform(transforms)
    timer = node.create_timer(0.05, broadcast, callback_group=group)
    status = []
    subscription = node.create_subscription(AgvStatus, "/agv/status", lambda m: status.append(m), 10)
    client = ActionClient(node, FollowEdge, "/agv/follow_edge", callback_group=group)
    control = node.create_client(SetControl, "/agv/set_control", callback_group=group)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    process = None
    try:
        with tempfile.TemporaryDirectory(prefix="agv-admission-") as directory:
            with open(Path(directory) / "bridge.log", "w+") as log:
                process = subprocess.Popen([sys.argv[1], "--ros-args", "-p", "use_sim_time:=false",
                    "-p", "manage_navigation:=false", "-p", "require_battery_state:=false",
                    "-p", "maps_dir:=" + directory, "-p", "robot_package:=simulated_chassis",
                    "-p", "spin_progress_timeout_sec:=20.0"], stdout=log, stderr=subprocess.STDOUT)
                try:
                    assert client.wait_for_server(timeout_sec=15)
                    wait_until(lambda: status and status[-1].pose_initialized)
                    def send(command):
                        goal = FollowEdge.Goal()
                        goal.command_id = command
                        goal.node_id = "test-target"
                        goal.edge_id = "test-edge"
                        goal.edge_type = "STRAIGHT"
                        goal.x = 0.0
                        goal.y = 2.0
                        goal.max_speed = 0.2
                        goal.step = 0.1
                        goal.end_point = True
                        h = value(client.send_goal_async(goal))
                        assert h.accepted, "Business failure must carry a result, not transport REJECT"
                        return h
                    old = send("occupied-old")
                    wait_until(lambda: status[-1].active_command_id == "occupied-old" and status[-1].state == "PRE_ROTATING")
                    new = send("refused-new")
                    result = value(new.get_result_async())
                    assert result.status == 6 and not result.result.success
                    assert result.result.command_id == "refused-new"
                    assert "occupied-old" in result.result.message and "PRE_ROTATING" in result.result.message
                    assert status[-1].active_command_id == "occupied-old"
                    assert value(old.cancel_goal_async()).goals_canceling
                    assert value(old.get_result_async()).status == 5
                    wait_until(lambda: control.service_is_ready())
                    req = SetControl.Request()
                    req.action = "stop"
                    assert value(control.call_async(req)).success
                    stopped = value(send("stopped-new").get_result_async())
                    assert stopped.status == 6 and "stop" in stopped.result.message
                    req.action = "start"
                    assert value(control.call_async(req)).success
                    complete_actions.set()
                    def complete(command, mode, x, y, backup=False, failure=None):
                        abort_cause[0] = failure
                        goal = FollowEdge.Goal()
                        goal.command_id = command
                        goal.node_id = "completed-target"
                        goal.edge_type = mode
                        goal.edge_id = "" if mode == "FREE" else "edge-complete"
                        goal.x, goal.y = x, y
                        goal.theta = 1.0
                        goal.max_speed = 0.2
                        goal.step = 0.2
                        goal.end_point = not backup
                        goal.back_up = backup
                        if mode == "CURVE":
                            from geometry_msgs.msg import Point
                            goal.control_points = [Point(x=1.0, y=1.0)]
                        handle = value(client.send_goal_async(goal))
                        assert handle.accepted
                        result = value(handle.get_result_async())
                        if failure is None:
                            assert result.status == 4 and result.result.success, result
                        else:
                            assert result.status == 6 and not result.result.success, result
                            if failure:
                                assert failure in result.result.message, result
                            else:
                                assert "未捕获本次动作" in result.result.message, result
                                assert "synthetic TF" not in result.result.message, result
                        assert result.result.command_id == command
                        assert result.result.node_id == "completed-target"
                    complete("complete-free", "FREE", 1.0, 0.0)
                    before = len(spin_targets)
                    complete("complete-straight", "STRAIGHT", 0.0, 2.0)
                    assert len(spin_targets) == before + 2, "straight must pre-rotate and finish-rotate"
                    complete("complete-curve", "CURVE", 2.0, 0.0)
                    assert any(p.pose.position.y > 0 for p in completed_paths[-1].poses)
                    before = len(spin_targets)
                    complete("complete-backup", "STRAIGHT", -1.0, 0.0, backup=True)
                    assert len(spin_targets) == before, "backup must not request spin"
                    wait_until(lambda: error_publisher.get_subscription_count() > 0)
                    complete("abort-path", "STRAIGHT", 1.0, 0.0, failure="synthetic TF: extrapolation into the future")
                    complete("abort-free", "FREE", 1.0, 0.0, failure="synthetic TF: cannot transform robot pose")
                    complete("abort-without-logs", "FREE", 1.0, 0.0, failure="")
                    print("PASS: path/free failures carry correlated cause in action result; next action excludes previous errors")
                    print("PASS: busy request returns old ID/stage without replacing old task; stop returns explicit ABORTED")
                    print("PASS: FREE/straight/curve/backup complete through real Manager callbacks with fake Nav2")
                except Exception:
                    log.flush()
                    log.seek(0)
                    print(log.read(), file=sys.stderr)
                    raise
                finally:
                    shutting_down.set()
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=8)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait(timeout=3)
    finally:
        shutting_down.set()
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        for server in servers:
            server.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
