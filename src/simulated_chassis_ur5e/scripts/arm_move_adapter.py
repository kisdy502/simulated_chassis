#!/usr/bin/env python3
"""/agv/arm_move → UR5e JTC 薄适配(《机械臂控制接口对接指引》仿真侧实现)。

行为契约:
  - 受理即响应:service 回调只做校验并下发轨迹,不等执行完;
  - 执行 = 向 /joint_trajectory_controller/joint_trajectory 发两点轨迹
    (当前角 → 目标角,time_from_start=duration_sec),插值与限速由 JTC 保证;
  - joints[i] 对应该组关节名字典序第 i 个(契约 §3.2;
    UR5e 实际字典序 = elbow, shoulder_lift, shoulder_pan, wrist_1..3);
  - 单臂宽松匹配:left/right 均驱动 UR5e;waist 拒绝(未装备);
  - 执行中收到新指令:新轨迹直接替换(JTC 语义),返回成功;
  - duration_sec<=0/缺省 → 3s;限位超限截断不报错。
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from agv_bridge_v2_interfaces.srv import ArmMove

# /joint_states 中 UR5e 六关节字典序(JTC 的 state 关节同样适用)
ARM_JOINTS = sorted([
    'ur5e_shoulder_pan_joint', 'ur5e_shoulder_lift_joint', 'ur5e_elbow_joint',
    'ur5e_wrist_1_joint', 'ur5e_wrist_2_joint', 'ur5e_wrist_3_joint',
])
# URDF 限位(超限截断):pan/lift/wrist ±2π,elbow ±π
LIMITS = {
    'ur5e_elbow_joint': (-math.pi, math.pi),
}
DEFAULT_DURATION = 3.0


class ArmMoveAdapter(Node):

    def __init__(self):
        super().__init__('arm_move_adapter')
        self._lock = threading.Lock()
        self._current = {}  # name -> position
        self._js_sub = self.create_subscription(
            JointState, '/joint_states', self._on_joints, qos_profile_sensor_data)
        self._traj_pub = self.create_publisher(
            JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self._srv = self.create_service(ArmMove, '/agv/arm_move', self._handle)
        self.get_logger().info(
            f'/agv/arm_move 就绪:关节(字典序)={ARM_JOINTS},目标话题='
            '/joint_trajectory_controller/joint_trajectory')

    def _on_joints(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINTS_SET:
                    self._current[name] = float(pos)

    def _handle(self, request, response):
        # §5 单臂宽松匹配
        if request.arm not in ('left', 'right'):
            response.success = False
            response.message = (f"waist not equipped" if request.arm == 'waist'
                                else f"arm 非法: '{request.arm}'(仅支持 left | right | waist)")
            return response
        joints = list(request.joints)
        if len(joints) != len(ARM_JOINTS):
            response.success = False
            response.message = f'expected {len(ARM_JOINTS)} joints'
            return response

        with self._lock:
            current = dict(self._current)
        if not all(n in current for n in ARM_JOINTS):
            response.success = False
            response.message = 'joint states unavailable(尚未收到 /joint_states)'
            return response

        duration = float(request.duration_sec) if request.duration_sec > 0 else DEFAULT_DURATION
        duration = min(max(duration, 0.5), 300.0)

        # 限位截断(契约 §3.3:超限截断不必报错)
        target = []
        for name, value in zip(ARM_JOINTS, joints):
            lo, hi = LIMITS.get(name, (-2.0 * math.pi, 2.0 * math.pi))
            target.append(min(max(float(value), lo), hi))

        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        p0 = JointTrajectoryPoint()
        p0.positions = [current[n] for n in ARM_JOINTS]
        p0.time_from_start.sec = 0
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        traj.points = [p0, p1]
        self._traj_pub.publish(traj)

        response.success = True
        response.message = 'accepted'
        self.get_logger().info(
            f'arm_move[{request.arm}→ur5e] {duration:.1f}s: '
            + ', '.join(f'{n.replace("ur5e_", "")}={v:+.3f}' for n, v in zip(ARM_JOINTS, target)))
        return response


ARM_JOINTS_SET = set(ARM_JOINTS)


def main():
    rclpy.init()
    node = ArmMoveAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
