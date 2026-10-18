#!/usr/bin/env python3
"""双臂+腰部上装仿真节点(F1)。

职责:
  1. 订阅 /joint_states,过滤底盘舵轮/轮子消息,与上装(腰+双臂)关节
     合并后以 rate(Hz) 重发 —— 上位机在一条消息里拿全全部关节;
     自过滤规则:消息内含 arm/waist 关节名则丢弃(那是自己发的,防回环)。
  2. 服务 /agv/arm_move(agv_bridge_v2_interfaces/srv/ArmMove):
     受理后 s 型(0.5*(1-cos(pi*t/T)))插值执行,进度经 /joint_states 体现;
     同组忙时拒绝;不改动 /agv/status 的 mode。

关节命名契约(上位机 RobotLayer.ts / RMS pose/query):
  舵轮含 steer(底盘自带 wheel_*_steering_joint ✓)
  左臂 arm_left_joint_1..7 / 右臂 arm_right_joint_1..7 / 腰 waist_joint_1..2
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import JointState
from agv_bridge_v2_interfaces.srv import ArmMove

GROUPS = {
    'left': [f'arm_left_joint_{i}' for i in range(1, 8)],
    'right': [f'arm_right_joint_{i}' for i in range(1, 8)],
    'waist': ['waist_joint_1', 'waist_joint_2'],
}
ARM_JOINTS = [n for names in GROUPS.values() for n in names]
HOME_POSE = {n: 0.0 for n in ARM_JOINTS}


class DualArmSimNode(Node):

    def __init__(self):
        super().__init__('dual_arm_sim_node')
        self.declare_parameter('rate', 20.0)
        rate = float(self.get_parameter('rate').value)

        self._lock = threading.Lock()
        self._chassis = JointState()          # 最近一帧底盘关节
        self._arm = dict(HOME_POSE)           # 当前上装关节角
        self._motions = {}                    # group -> (start, target, t0, dur)

        self._js_pub = self.create_publisher(JointState, '/joint_states', 20)
        self._arm_srv = self.create_service(ArmMove, '/agv/arm_move', self._handle_arm_move)
        # 底盘 joint_state_broadcaster 默认 RELIABLE;best_effort 订阅兼容两者
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._js_sub = self.create_subscription(JointState, '/joint_states', self._on_joint_states, qos)

        self.create_timer(1.0 / max(rate, 1.0), self._tick)
        self.get_logger().info(
            f'上装关节数 {len(ARM_JOINTS)}(左7+右7+腰2),/joint_states 合并 {rate:.0f}Hz,/agv/arm_move 就绪')

    # ---- /joint_states 合并转发 ----
    def _on_joint_states(self, msg: JointState):
        if any(('arm' in n or 'waist' in n) for n in msg.name):
            return  # 自己发布的合并帧(或重复帧),跳过防回环
        with self._lock:
            self._chassis = msg

    def _tick(self):
        now = self.get_clock().now()
        wall = now.nanoseconds * 1e-9
        with self._lock:
            for group, motion in list(self._motions.items()):
                start, target, t0, dur = motion
                t = min((wall - t0) / dur, 1.0)
                s = 0.5 * (1.0 - math.cos(math.pi * t))
                for name, a, b in zip(GROUPS[group], start, target):
                    self._arm[name] = a + (b - a) * s
                if t >= 1.0:
                    del self._motions[group]
            chassis = self._chassis
            arm = dict(self._arm)

        out = JointState()
        out.header.stamp = now.to_msg()
        out.name = list(chassis.name) + sorted(arm.keys())
        out.position = list(chassis.position) + [arm[n] for n in sorted(arm.keys())]
        out.velocity = list(getattr(chassis, 'velocity', [])) + [0.0] * len(arm)
        out.effort = []
        self._js_pub.publish(out)

    # ---- /agv/arm_move ----
    def _handle_arm_move(self, request, response):
        group = request.arm
        if group not in GROUPS:
            response.success = False
            response.message = f"arm 非法: '{group}'(仅支持 left | right | waist)"
            return response
        joints = list(request.joints)
        if len(joints) != len(GROUPS[group]):
            response.success = False
            response.message = f'{group} 需要 {len(GROUPS[group])} 个关节角,收到 {len(joints)}'
            return response
        dur = float(request.duration_sec)
        if not (0.05 <= dur <= 300.0):
            response.success = False
            response.message = f'duration_sec 需在 [0.05, 300],收到 {dur}'
            return response

        wall = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            if group in self._motions:
                response.success = False
                response.message = f'{group} 上一条指令仍在执行,请稍后'
                return response
            start = [self._arm[n] for n in GROUPS[group]]
            self._motions[group] = (start, joints, wall, dur)

        names = GROUPS[group]
        self.get_logger().info(
            f'arm_move[{group}] 受理: ' + ', '.join(f'{n}={v:.3f}' for n, v in zip(names, joints))
            + f', {dur:.1f}s')
        response.success = True
        response.message = '已受理,异步插值执行中'
        return response


def main():
    rclpy.init()
    node = DualArmSimNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
