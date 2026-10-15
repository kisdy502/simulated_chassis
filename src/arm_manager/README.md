# arm_manager

独立 ROS 2 Humble 节点，通过 MoveIt2 的 `/move_action`（只规划）及 `/execute_trajectory`（执行）管理机械臂。支持关节目标、末端位姿目标、关节示教点保存/调用、取消、状态反馈和可选的模拟故障注入。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arm_manager_interfaces arm_manager simulated_ur5e
source install/setup.bash
ros2 launch simulated_ur5e ur5e.launch.py mode:=mock allow_fault_injection:=true
```

UR5e 启动默认包含 manager；`include_arm_manager:=false` 可关闭。Fortress 启动也可使用，但仍需要 simulated_ur5e README 中的官方仿真依赖。仿真同一时间只允许一个机械臂系统使用默认话题。

对已启动的 MoveIt 系统，可单独启动（不要与 UR5e 自带 manager 重复启动）：

```bash
ros2 launch arm_manager arm_manager.launch.py simulation_mode:=true allow_fault_injection:=true
```

独立启动默认禁止故障注入与仿真复位。更换机械臂需调整 `config` 中规划组、末端 link、关节名称及 MoveIt action 路径，并使用独立示教文件。

## 上位机接口

| 路径 | 类型 | 功能 |
|---|---|---|
| `/arm_manager/command` | `arm_manager_interfaces/action/ArmCommand` | joint / pose / named 运动，反馈、结果、Action 取消 |
| `/arm_manager/status` | `arm_manager_interfaces/msg/ArmStatus` | 5 Hz 状态，transient-local；ready、busy、故障、关节位置 |
| `/arm_manager/save_teach_point` | `arm_manager_interfaces/srv/SaveTeachPoint` | 从最新完整关节状态保存示教点 |
| `/arm_manager/list_teach_points` | `arm_manager_interfaces/srv/ListTeachPoints` | 列出已保存点与关节角 |
| `/arm_manager/delete_teach_point` | `arm_manager_interfaces/srv/DeleteTeachPoint` | 删除示教点 |
| `/arm_manager/cancel` | `std_srvs/srv/Trigger` | 请求取消当前任务；应等待最终结果确认结束 |
| `/arm_manager/inject_fault` | `arm_manager_interfaces/srv/InjectFault` | 模拟故障并取消当前任务，默认禁用 |
| `/arm_manager/reset_fault` | `std_srvs/srv/Trigger` | 仅仿真软件复位，不移动机械臂 |

运动参数必须提供唯一 `command_id`、操作类型、速度/加速度比例 `(0,1]`、正的总超时秒数。关节角单位为 rad，位置为 m，姿态为单位四元数。`pose` 必须提供明确的坐标系；表示末端目标的避碰规划，不保证直线运动。

```bash
# 等待 ready: true
ros2 topic echo /arm_manager/status

# 保存当前位置为 ready_pose
ros2 service call /arm_manager/save_teach_point arm_manager_interfaces/srv/SaveTeachPoint \
  "{name: ready_pose, overwrite: false}"
ros2 service call /arm_manager/list_teach_points arm_manager_interfaces/srv/ListTeachPoints '{}'

# 调用示教点
ros2 action send_goal /arm_manager/command arm_manager_interfaces/action/ArmCommand \
  "{command_id: named-001, operation: named, target_name: ready_pose, velocity_scale: 0.2, acceleration_scale: 0.2, timeout_sec: 30.0}" --feedback

# 关节目标（示例，先在仿真中确认工作空间）
ros2 action send_goal /arm_manager/command arm_manager_interfaces/action/ArmCommand \
  "{command_id: joint-001, operation: joint, joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint], joint_positions: [0.2, -1.57, 0.0, -1.57, 0.0, 0.0], velocity_scale: 0.2, acceleration_scale: 0.2, timeout_sec: 30.0}" --feedback

# 末端目标（可达性由实际姿态、场景及 IK 决定）
ros2 action send_goal /arm_manager/command arm_manager_interfaces/action/ArmCommand \
  "{command_id: pose-001, operation: pose, target_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.4, y: 0.2, z: 0.5}, orientation: {w: 1.0}}}, velocity_scale: 0.1, acceleration_scale: 0.1, timeout_sec: 30.0}" --feedback

ros2 service call /arm_manager/cancel std_srvs/srv/Trigger '{}'
ros2 service call /arm_manager/inject_fault arm_manager_interfaces/srv/InjectFault "{reason: simulated_collision}"
ros2 service call /arm_manager/reset_fault std_srvs/srv/Trigger '{}'
```

## 状态及恢复语义

正常流程：`idle → planning → executing → succeeded`。终态保留在 status 中，便于上位机读取；下次任务进入 planning。取消后为 canceled，参数/规划/超时失败为 failed，执行故障、注入故障或停止未确认会锁存为 fault。ready 表示可接受新运动任务，busy 表示管理器尚在处理当前任务。

只允许一个在途任务；忙、故障锁存、停止未确认、重复 ID 的请求会在 Action 接收阶段拒绝。其余非法参数会返回 INVALID_ARGUMENT。ID 去重范围为本节点生命周期，重启后清空；如需跨重启幂等，上位机应持久化任务记录。无进度百分比，反馈真实的 planning/executing 阶段，避免伪造进度。

总超时和停止确认超时使用单调墙上时钟，暂停仿真时仍生效。任务取消会取消下游 Action；执行阶段还会发布 MoveIt 的 `/trajectory_execution_event` stop 事件，兼容 Humble ExecuteTrajectory 的取消行为。停止确认最多额外等待 stop_timeout。超过停止确认时间时返回 STOP_UNCONFIRMED 并锁住新任务，直到下游终态被确认。这里确认的是下游任务终态，不是安全级物理停止确认。无法确认的通信故障需要人工核对系统状态；复位不能绕过这把锁。

原型包也可以使用同一接口（先单独启动原型的 MoveIt）：

```bash
ros2 launch arm_manager arm_manager.launch.py \
  config:=$(ros2 pkg prefix --share arm_manager)/config/prototype.yaml \
  simulation_mode:=true teach_points_file:=~/.ros/arm_manager/prototype_teach_points.json
```

示教点保存的是完整的实际关节状态，要求管理器空闲、无故障、状态新鲜。JSON 文件默认 `~/.ros/arm_manager/ur5e_teach_points.json`，写入采用临时文件原子替换，重启保留，旧点覆盖须显式授权。损坏文件或不匹配机械臂的文件会拒绝启动，不静默覆盖。

这是任务编排层，不是安全控制器：故障注入不代表真实碰撞检测；复位不会解锁 UR 保护停机、急停，也不会回 home。真实机器人仍需厂家状态与恢复适配。MoveIt 只检查已知规划场景，默认不提供完整动态碰撞防护；Gazebo 障碍物也不会自动变成 MoveIt 障碍物。

管理器只仲裁经本接口发出的任务。运行任务时不要同时通过 RViz Execute、底层轨迹 Action 或其他程序控制同一机械臂；它们不经过该节点的单任务锁。当前版本未加入夹爪、直线/圆弧运动、拖动示教或底盘停车联锁。

## 验证

在工作空间根目录、source 后执行：

```bash
python3 src/arm_manager/test/integration_mock.py
python3 src/arm_manager/test/stop_semantics.py
```

第一项启动实际 UR5e mock + MoveIt（ROS_DOMAIN_ID=89，无 RViz），验证关节/末端/示教运动、场景碰撞规划失败、忙与重复拒绝、Action/Service 取消、超时、故障恢复。第二项使用故意延迟应答或不响应取消的假后端（ROS_DOMAIN_ID=91），验证停止未确认时的锁、晚到目标取消、示教重启恢复、真机模式禁用模拟恢复、陈旧状态拒绝。均使用临时示教文件，不修改用户已保存的点。
