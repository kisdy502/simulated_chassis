# 自由绕障到点（FREE）

沿用 /agv/follow_edge，action 类型 agv_bridge_v2_interfaces/action/FollowEdge。
edge_type=FREE 时提交 Nav2 NavigateToPose，由其行为树负责全局规划、路径跟踪、重新规划和恢复。
STRAIGHT/CURVE 的原有路径执行逻辑不变。未知的非空 edge_type 明确失败，不再退化为直线。

## WebSocket 示例

~~~json
{
  "op": "send_action_goal",
  "id": "free_001",
  "action": "/agv/follow_edge",
  "action_type": "agv_bridge_v2_interfaces/action/FollowEdge",
  "feedback": true,
  "args": {
    "command_id": "free_001",
    "node_id": "P001",
    "edge_type": "FREE",
    "x": 5.0,
    "y": 2.0,
    "theta": 0.0,
    "max_speed": 0.6,
    "back_up": false,
    "reverse": false,
    "control_points": []
  }
}
~~~

## 参数及行为

- x/y：map 坐标系，单位米；theta：最终朝向，单位弧度，均要求有限数值。
- max_speed：正数时下发 /speed_limit（绝对速度 m/s）；<=0 沿用原接口默认 1.0m/s。
  当前复用既有限速机制；已知 Humble MPPI 优化器重置可能覆盖任务限速，本改动不保证修复该问题。
- back_up/reverse 必须为 false，control_points 必须为空；冲突参数返回失败结果。
- edge_id/source_id/target_id 可留空，不根据边起点推导反向行驶。
- step 不参与路径生成；end_point 不控制末端旋转，FREE 始终以 theta 为目标朝向。
- 不执行 bridge 预旋转或额外末端旋转，旋转与恢复行为由 Nav2 行为树控制。
- edge_type 必填且区分大小写；不再根据控制点推断，只有显式 FREE 启用自主规划。

## 反馈、结果与取消

反馈继续提供 x/y/theta/state，执行时 state=EXECUTING。
Nav2 成功、终止、拒绝及取消通过现有 success/message/command_id/node_id 结果返回。
取消继续调用 /agv/follow_edge 的 cancel_action_goal，等待底层动作停止；
响应或取消超时仍执行已有的停止确认保护，不自动重放任务。

## 前提与验证

定位、TF、地图与 Nav2 生命周期必须就绪，navigate_to_pose 动作服务必须可用。
障碍物应进入代价地图，且存在可通行绕行路线；FREE 不承诺所有障碍均可绕过。
验证时观察 /plan 是否绕开全局障碍，检查局部代价地图的障碍感知，
并测试无路可达、执行取消后新任务及任务速度限制。
单元测试使用隔离 ROS domain 的模拟动作服务器，不向真实机器人发送目标。


## 所有导航模式的严格校验

| edge_type | edge_id | control_points |
| --- | --- | --- |
| FREE | 不校验，可省略或为空 | 必须为空 |
| STRAIGHT | 必须非空且不能仅含空白 | 必须为空 |
| CURVE | 必须非空且不能仅含空白 | 必须为 1 或 2 个，各点 x/y 为有限数值 |
| ELEVATION | 必须非空且不能仅含空白 | 必须为空 |

空或未知 edge_type 直接失败，曲线参数错误不会转成直线。
x/y/theta/max_speed/step 必须为有限数值。max_speed、step 的非正数仍沿用默认值策略。
倒车仍使用 STRAIGHT 的 back_up=true，FREE 禁止 back_up=true 或 reverse=true。
参数校验先于定位与 Nav2 就绪检查，返回第一个错误；修正后可重新提交。

业务校验失败通过 action_result 返回 status=6（ABORTED），外层 result=true，
values.success=false，values.message 明确指出字段和修复要求，command_id/node_id 原样回传。
例如：参数错误：CURVE 的 control_points 数量为 0；必须提供 1 个（二阶）或 2 个（三阶）控制点，不会退化为直线。

ROS 消息字段不接受 JSON null，应省略可选字段或传对应类型的空值。
rosbridge 类型转换失败时，任务不会到达 C++：外层 result=false，status=0，values 是错误字符串。
省略 x/y/theta 会得到 ROS 默认 0，C++ 无法区别省略与显式 0；上位机必须检查坐标已填写。
