#!/usr/bin/env python3
import rclpy
from action_msgs.srv import CancelGoal

rclpy.init()
node = rclpy.create_node('cancel_nav')
cli = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
if not cli.wait_for_service(timeout_sec=5.0):
    print('navigate_to_pose not found'); raise SystemExit
future = cli.call_async(CancelGoal.Request())   # 空 goal_id = 取消所有目标
rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
print('cancel result:', future.result().return_code, future.result().goals_canceling)