#!/usr/bin/env python3
"""Single-flight arm tasks. Wall-clock deadlines remain valid with paused /clock."""
import copy
import json
import math
import os
import re
import signal
import tempfile
import threading
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import Clock, ClockType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from action_msgs.msg import GoalStatus
from arm_manager_interfaces.action import ArmCommand
from arm_manager_interfaces.msg import ArmStatus, TeachPoint
from arm_manager_interfaces.srv import SaveTeachPoint, ListTeachPoints, DeleteTeachPoint, InjectFault
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint, PositionConstraint, OrientationConstraint, MoveItErrorCodes
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from std_msgs.msg import String


class TaskError(Exception):
    def __init__(self, code, message, moveit_code=0):
        super().__init__(message)
        self.code, self.moveit_code = code, moveit_code


class ArmManager(Node):
    def __init__(self):
        super().__init__('arm_manager')
        defaults = {
            'group_name': 'ur_manipulator', 'tool_link': 'tool0',
            'joint_names': ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'],
            'move_action': '/move_action', 'execute_action': '/execute_trajectory',
            'execution_event_topic': '/trajectory_execution_event',
            'joint_states_topic': '/joint_states', 'planning_time': 5.0,
            'joint_state_timeout': 2.0, 'stop_timeout': 5.0,
            'simulation_mode': False, 'allow_fault_injection': False,
            'teach_points_file': '~/.ros/arm_manager/ur5e_teach_points.json',
        }
        self.cfg = {key: self.declare_parameter(key, value).value for key, value in defaults.items()}
        self.names = list(self.cfg['joint_names'])
        if not self.names or len(set(self.names)) != len(self.names):
            raise ValueError('joint_names must be nonempty and unique')
        for key in ('planning_time', 'joint_state_timeout', 'stop_timeout'):
            if not math.isfinite(self.cfg[key]) or self.cfg[key] <= 0:
                raise ValueError(f'{key} must be finite and > 0')
        self.lock = threading.RLock()
        self.busy = False
        self.fault = False
        self.state, self.code, self.detail, self.command_id = 'idle', '', 'No active task; inspect ready for availability', ''
        self.joints, self.joint_time = {}, 0.0
        self.used_ids = set()
        self.pending = set()  # send requests stay tracked until a terminal downstream result
        self.pending_executions = set()
        self.outer = None
        self.interrupt = threading.Event()
        self.interrupt_code, self.interrupt_detail = '', ''
        self.path = Path(os.path.expanduser(self.cfg['teach_points_file']))
        self.points = self.load_points()
        group = ReentrantCallbackGroup()
        self.planner = ActionClient(self, MoveGroup, self.cfg['move_action'], callback_group=group)
        self.executor_client = ActionClient(self, ExecuteTrajectory, self.cfg['execute_action'], callback_group=group)
        self.stop_pub = self.create_publisher(String, self.cfg['execution_event_topic'], 10)
        self.server = ActionServer(self, ArmCommand, '~/command', self.execute,
                                   goal_callback=self.accept, cancel_callback=self.cancel_goal,
                                   callback_group=group)
        self.create_subscription(JointState, self.cfg['joint_states_topic'], self.on_joints,
                                 qos_profile_sensor_data, callback_group=group)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(ArmStatus, '~/status', qos)
        self.create_service(SaveTeachPoint, '~/save_teach_point', self.save_point, callback_group=group)
        self.create_service(ListTeachPoints, '~/list_teach_points', self.list_points, callback_group=group)
        self.create_service(DeleteTeachPoint, '~/delete_teach_point', self.delete_point, callback_group=group)
        self.create_service(InjectFault, '~/inject_fault', self.inject_fault, callback_group=group)
        self.create_service(Trigger, '~/reset_fault', self.reset_fault, callback_group=group)
        self.create_service(Trigger, '~/cancel', self.cancel_service, callback_group=group)
        self.create_timer(0.2, self.publish_status, callback_group=group,
                          clock=Clock(clock_type=ClockType.STEADY_TIME))

    def load_points(self):
        if not self.path.exists():
            return {}
        # Do not silently overwrite a damaged file or points from another arm.
        document = json.loads(self.path.read_text(encoding='utf-8'))
        if document.get('version') != 1 or document.get('joint_names') != self.names:
            raise ValueError(f'Incompatible teach point file: {self.path}')
        points = document['points']
        if not isinstance(points, dict):
            raise ValueError('Teach points must be an object')
        for name, values in points.items():
            if not self.valid_name(name) or len(values) != len(self.names) or not all(
                    isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in values):
                raise ValueError(f'Invalid teach point: {name}')
        return points

    @staticmethod
    def valid_name(name):
        return isinstance(name, str) and re.fullmatch(r'[A-Za-z0-9_\-]{1,64}', name) is not None

    def write_points(self, points):
        self.path.parent.mkdir(parents=True, exist_ok=True)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', encoding='utf-8', dir=self.path.parent,
                                             prefix='.teach-', delete=False) as stream:
                temporary = stream.name
                json.dump({'version': 1, 'joint_names': self.names, 'points': points}, stream,
                          ensure_ascii=False, indent=2, allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    def on_joints(self, message):
        if len(message.name) != len(message.position) or len(set(message.name)) != len(message.name):
            return
        values = dict(zip(message.name, message.position))
        # Never treat a partial message as a fresh complete robot state.
        if all(name in values and math.isfinite(values[name]) for name in self.names):
            with self.lock:
                self.joints = {name: values[name] for name in self.names}
                self.joint_time = time.monotonic()

    def fresh(self):
        return bool(self.joints) and time.monotonic() - self.joint_time < self.cfg['joint_state_timeout']

    def ready(self):
        return not self.busy and not self.fault and not self.pending and self.fresh() and \
            self.planner.server_is_ready() and self.executor_client.server_is_ready()

    def publish_status(self):
        with self.lock:
            retry_stop = self.fault and bool(self.pending_executions)
            status = ArmStatus()
            status.header.stamp = self.get_clock().now().to_msg()
            status.state, status.command_id = self.state, self.command_id
            status.busy, status.ready, status.fault_latched = self.busy, self.ready(), self.fault
            status.stop_unconfirmed = bool(self.pending) and not self.busy
            status.error_code, status.message = self.code, self.detail
            status.joint_names = self.names
            status.joint_positions = [self.joints[name] for name in self.names] if self.joints else []
        self.pub.publish(status)
        if retry_stop:
            self.stop_pub.publish(String(data='stop'))

    def transition(self, state, code='', detail=''):
        with self.lock:
            if self.fault and state in ('planning', 'executing', 'succeeded'):
                raise TaskError(self.interrupt_code or 'INTERNAL_ERROR', self.interrupt_detail or 'Fault latched')
            self.state, self.code, self.detail = state, code, detail
            outer = self.outer
        if outer is not None and outer.is_active:
            feedback = ArmCommand.Feedback(command_id=self.command_id, state=state, detail=detail)
            outer.publish_feedback(feedback)
        self.publish_status()

    def accept(self, request):
        with self.lock:
            if self.busy or self.fault or self.pending or not request.command_id.strip() or \
                    len(request.command_id) > 128 or request.command_id in self.used_ids:
                self.get_logger().warning('Command rejected: busy/fault/duplicate/invalid ID; inspect status')
                return GoalResponse.REJECT
            self.busy = True  # reserve here, before the execution callback is scheduled
            self.used_ids.add(request.command_id)
            self.command_id = request.command_id
            self.interrupt.clear()
            self.interrupt_code, self.interrupt_detail = '', ''
        return GoalResponse.ACCEPT

    def request_stop(self, code, detail):
        with self.lock:
            # An injected fault takes priority over a simultaneous user cancel.
            if not self.interrupt.is_set() or code == 'INJECTED_FAULT':
                self.interrupt_code, self.interrupt_detail = code, detail
            self.interrupt.set()

    def cancel_goal(self, handle):
        self.request_stop('CANCELED', 'Canceled by action client')
        return CancelResponse.ACCEPT

    def cancel_service(self, request, response):
        with self.lock:
            if not self.busy:
                response.success, response.message = False, 'No active task'
            else:
                self.request_stop('CANCELED', 'Canceled by service; wait for terminal task result')
                response.success, response.message = True, 'Cancel requested; stopping is asynchronous'
        return response

    def inject_fault(self, request, response):
        with self.lock:
            if not (self.cfg['simulation_mode'] and self.cfg['allow_fault_injection']):
                response.success, response.message = False, 'Fault injection is disabled'
                return response
            self.fault = True
            reason = request.reason.strip() or 'Simulated fault'
            self.request_stop('INJECTED_FAULT', reason)
            self.state, self.code, self.detail = 'fault', 'INJECTED_FAULT', reason
            response.success, response.message = True, 'Fault latched; active task cancellation requested'
        self.publish_status()
        return response

    def reset_fault(self, request, response):
        with self.lock:
            if not self.cfg['simulation_mode']:
                response.success, response.message = False, 'Simulation-only reset; hardware recovery is not implemented'
            elif self.busy or self.pending:
                response.success, response.message = False, 'Task active or downstream stop unconfirmed'
            elif not self.fresh():
                response.success, response.message = False, 'Joint state missing or stale'
            else:
                self.fault = False
                self.interrupt.clear()
                self.state, self.code, self.detail = 'idle', '', 'Software fault cleared; no motion commanded'
                response.success, response.message = True, self.detail
        self.publish_status()
        return response

    def save_point(self, request, response):
        with self.lock:
            if self.busy or self.fault or self.pending or not self.fresh():
                response.success, response.message = False, 'Requires idle, unfaulted, fresh joint state'
            elif not self.valid_name(request.name):
                response.success, response.message = False, 'Name must match [A-Za-z0-9_-], length 1..64'
            elif request.name in self.points and not request.overwrite:
                response.success, response.message = False, 'Point exists; explicitly set overwrite=true'
            else:
                updated = copy.deepcopy(self.points)
                updated[request.name] = [self.joints[name] for name in self.names]
                try:
                    self.write_points(updated)
                    self.points = updated
                    response.success, response.message = True, f'Saved {request.name}'
                except OSError as error:
                    response.success, response.message = False, f'Write failed: {error}'
        return response

    def list_points(self, request, response):
        with self.lock:
            response.points = [TeachPoint(name=name, joint_names=self.names,
                                joint_positions=list(values)) for name, values in sorted(self.points.items())]
        return response

    def delete_point(self, request, response):
        with self.lock:
            if self.busy or request.name not in self.points:
                response.success, response.message = False, 'Task active or point does not exist'
            else:
                updated = copy.deepcopy(self.points)
                del updated[request.name]
                try:
                    self.write_points(updated)
                    self.points = updated
                    response.success, response.message = True, f'Deleted {request.name}'
                except OSError as error:
                    response.success, response.message = False, f'Write failed: {error}'
        return response

    def constraints(self, request):
        for value in (request.velocity_scale, request.acceleration_scale):
            if not math.isfinite(value) or not 0 < value <= 1:
                raise TaskError('INVALID_ARGUMENT', 'Velocity and acceleration scales must be in (0, 1]')
        if not math.isfinite(request.timeout_sec) or request.timeout_sec <= 0:
            raise TaskError('INVALID_ARGUMENT', 'timeout_sec must be finite and > 0')
        result = Constraints()
        if request.operation in ('joint', 'named'):
            if request.operation == 'named':
                with self.lock:
                    if request.target_name not in self.points:
                        raise TaskError('INVALID_ARGUMENT', 'Unknown teach point')
                    names, values = self.names, list(self.points[request.target_name])
            else:
                names, values = list(request.joint_names), list(request.joint_positions)
            if len(names) != len(self.names) or set(names) != set(self.names) or \
                    len(values) != len(names) or not all(math.isfinite(value) for value in values):
                raise TaskError('INVALID_ARGUMENT', 'Target requires every configured joint once and finite angles')
            result.joint_constraints = [JointConstraint(joint_name=name, position=float(value),
                tolerance_above=0.001, tolerance_below=0.001, weight=1.0) for name, value in zip(names, values)]
        elif request.operation == 'pose':
            pose = copy.deepcopy(request.target_pose)
            q, p = pose.pose.orientation, pose.pose.position
            values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
            norm = math.sqrt(sum(value * value for value in (q.x, q.y, q.z, q.w)))
            if not pose.header.frame_id.strip() or not all(math.isfinite(v) for v in values) or \
                    not math.isfinite(norm) or abs(norm - 1.0) > 0.01:
                raise TaskError('INVALID_ARGUMENT', 'Pose needs explicit frame, finite coordinates and unit quaternion')
            q.x, q.y, q.z, q.w = q.x / norm, q.y / norm, q.z / norm, q.w / norm
            position = PositionConstraint(header=pose.header, link_name=self.cfg['tool_link'], weight=1.0)
            position.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.005])]
            position.constraint_region.primitive_poses = [pose.pose]
            orientation = OrientationConstraint(header=pose.header, link_name=self.cfg['tool_link'],
                orientation=q, absolute_x_axis_tolerance=0.01, absolute_y_axis_tolerance=0.01,
                absolute_z_axis_tolerance=0.01, weight=1.0)
            result.position_constraints, result.orientation_constraints = [position], [orientation]
        else:
            raise TaskError('INVALID_ARGUMENT', 'operation must be joint, pose or named')
        return result

    def checkpoint(self, deadline):
        if self.interrupt.is_set():
            raise TaskError(self.interrupt_code, self.interrupt_detail)
        if time.monotonic() >= deadline:
            raise TaskError('TIMEOUT', 'Task wall-clock deadline exceeded')
        if not rclpy.ok():
            raise TaskError('INTERNAL_ERROR', 'ROS is shutting down')

    def backend(self, client, goal, deadline):
        """Track late goal acknowledgments and do not release stop lock prematurely."""
        self.checkpoint(deadline)
        token = object()
        stop = threading.Event()
        tracking = {'result': None}
        with self.lock:
            self.pending.add(token)
            if client is self.executor_client:
                self.pending_executions.add(token)

        def finished(_):
            with self.lock:
                self.pending.discard(token)
                self.pending_executions.discard(token)

        def terminal(future):
            try:
                response = future.result()
                if response.status in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED,
                                       GoalStatus.STATUS_CANCELED):
                    finished(None)
            except Exception:
                self.get_logger().error('Backend result unavailable; stop cannot be confirmed')

        def accepted(future):
            try:
                handle = future.result()
                if not handle.accepted:
                    finished(None)
                    return
                result_future = handle.get_result_async()
                tracking['result'] = result_future
                result_future.add_done_callback(terminal)
                if stop.is_set() or self.interrupt.is_set():
                    handle.cancel_goal_async()
                    if client is self.executor_client:
                        self.stop_pub.publish(String(data='stop'))
            except Exception:
                # Unknown transport state remains locked; restart/reconcile backend explicitly.
                self.get_logger().error('Backend acknowledgment failed; stop cannot be confirmed')

        try:
            sent = client.send_goal_async(goal)
        except Exception:
            finished(None)  # send failed synchronously, so no goal exists
            raise
        sent.add_done_callback(accepted)
        handle = None
        result_future = None
        try:
            while not sent.done():
                self.checkpoint(deadline)
                time.sleep(0.02)
            handle = sent.result()
            if not handle.accepted:
                raise TaskError('NOT_READY', 'MoveIt rejected the downstream goal')
            while tracking['result'] is None:
                self.checkpoint(deadline)
                time.sleep(0.02)
            result_future = tracking['result']
            while not result_future.done():
                self.checkpoint(deadline)
                if client is self.executor_client and not self.fresh():
                    raise TaskError('EXECUTION_FAILED', 'Joint feedback became stale; stopping execution')
                time.sleep(0.02)
            self.checkpoint(deadline)
            return result_future.result()
        except Exception as error:
            stop.set()
            stop_deadline = time.monotonic() + self.cfg['stop_timeout']
            # Acceptance might arrive after timeout. The callback cancels that late goal too.
            if handle is None and sent.done():
                try:
                    handle = sent.result()
                except Exception:
                    pass
            if handle is not None and handle.accepted:
                handle.cancel_goal_async()
            last_stop = 0.0
            while time.monotonic() < stop_deadline and rclpy.ok():
                with self.lock:
                    if token not in self.pending:
                        break
                # Humble ExecuteTrajectory cancellation alone does not preempt execution.
                # Repeat the TEM event to cover stop arriving before trajectory startup.
                if client is self.executor_client and time.monotonic() - last_stop >= 0.1:
                    self.stop_pub.publish(String(data='stop'))
                    last_stop = time.monotonic()
                time.sleep(0.02)
            with self.lock:
                pending = token in self.pending
            if pending:
                raise TaskError('STOP_UNCONFIRMED', f'{error}; downstream termination not confirmed') from error
            raise

    def execute(self, handle):
        result = ArmCommand.Result(command_id=handle.request.command_id)
        with self.lock:
            self.outer = handle
        try:
            constraints = self.constraints(handle.request)
            deadline = time.monotonic() + handle.request.timeout_sec
            self.checkpoint(deadline)
            with self.lock:
                if not self.fresh() or not self.planner.server_is_ready() or not self.executor_client.server_is_ready():
                    raise TaskError('NOT_READY', 'Requires fresh joint states and MoveIt action servers')
                start_names, start_positions = self.names, [self.joints[name] for name in self.names]
            goal = MoveGroup.Goal()
            goal.request.group_name = self.cfg['group_name']
            goal.request.pipeline_id = 'ompl'
            goal.request.num_planning_attempts = 1
            goal.request.allowed_planning_time = min(self.cfg['planning_time'], handle.request.timeout_sec)
            goal.request.max_velocity_scaling_factor = handle.request.velocity_scale
            goal.request.max_acceleration_scaling_factor = handle.request.acceleration_scale
            goal.request.start_state.is_diff = True
            goal.request.start_state.joint_state = JointState(name=start_names, position=start_positions)
            goal.request.goal_constraints = [constraints]
            goal.planning_options.plan_only = True
            self.transition('planning', detail='Computing collision-checked OMPL path')
            planned = self.backend(self.planner, goal, deadline)
            if planned.status != GoalStatus.STATUS_SUCCEEDED or planned.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise TaskError('PLANNING_FAILED', 'MoveIt planning failed', planned.result.error_code.val)
            self.checkpoint(deadline)
            trajectory = planned.result.planned_trajectory
            if not trajectory.joint_trajectory.points:
                raise TaskError('PLANNING_FAILED', 'MoveIt returned an empty trajectory')
            with self.lock:
                if not self.fresh():
                    raise TaskError('NOT_READY', 'Joint state became stale during planning')
            self.transition('executing', detail='Executing planned trajectory')
            executed = self.backend(self.executor_client, ExecuteTrajectory.Goal(trajectory=trajectory), deadline)
            if executed.status != GoalStatus.STATUS_SUCCEEDED or executed.result.error_code.val != MoveItErrorCodes.SUCCESS:
                raise TaskError('EXECUTION_FAILED', 'MoveIt execution failed', executed.result.error_code.val)
            with self.lock:
                self.checkpoint(deadline)
                if handle.is_cancel_requested:
                    raise TaskError('CANCELED', 'Canceled by action client')
                result.success, result.error_code, result.message = True, 'OK', 'Motion completed'
                result.moveit_error_code = MoveItErrorCodes.SUCCESS
                handle.succeed()
                self.transition('succeeded', 'OK', result.message)
        except Exception as error:
            if not isinstance(error, TaskError):
                self.get_logger().error(f'Task exception: {error}')
                error = TaskError('INTERNAL_ERROR', str(error))
            result.error_code, result.message, result.moveit_error_code = error.code, str(error), error.moveit_code
            with self.lock:
                # Failed planning / invalid input do not require protective-stop recovery.
                if error.code in ('INJECTED_FAULT', 'STOP_UNCONFIRMED', 'EXECUTION_FAILED', 'INTERNAL_ERROR'):
                    self.fault = True
                if self.fault:
                    terminal = 'fault'
                else:
                    terminal = 'canceled' if error.code == 'CANCELED' else 'failed'
                if handle.is_cancel_requested:
                    handle.canceled()
                else:
                    handle.abort()
                self.transition(terminal, error.code, str(error))
        finally:
            with self.lock:
                self.busy, self.outer = False, None
            self.publish_status()
        return result

    def destroy_node(self):
        self.server.destroy()
        self.planner.destroy()
        self.executor_client.destroy()
        return super().destroy_node()


def main():
    # Keep ROS alive long enough to send/receive downstream cancellation on exit.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = ArmManager()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    shutdown_requested = threading.Event()

    def shutdown_signal(signum, frame):
        node.request_stop('CANCELED', 'Node shutting down')
        shutdown_requested.set()

    signal.signal(signal.SIGINT, shutdown_signal)
    signal.signal(signal.SIGTERM, shutdown_signal)
    try:
        while rclpy.ok() and not shutdown_requested.is_set():
            executor.spin_once(timeout_sec=0.2)
    except KeyboardInterrupt:
        node.request_stop('CANCELED', 'Node shutting down')
    finally:
        deadline = time.monotonic() + node.cfg['stop_timeout'] + 1.0
        while rclpy.ok() and (node.busy or node.pending) and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        if node.pending:
            node.get_logger().error('Exiting with downstream stop unconfirmed; reconcile backend before restarting')
        executor.shutdown(timeout_sec=6.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
