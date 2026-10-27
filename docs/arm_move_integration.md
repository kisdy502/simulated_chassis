# UR5e 仿真机械臂接口对接说明

更新日期：2026-10-20。面向上位机开发；以当前 C++ `arm_move_adapter` 为准。

适用入口：`ros2 launch simulated_chassis_ur5e bridge.launch.py`。该 launch 自动启动 `/agv/arm_move` 服务、机械臂运行时及 rosbridge 白名单，无需单独启动适配节点。纯底盘通用 bridge 不提供机械臂执行器。`simulated_dual_arm` 是另一种 profile，不能把其关节数量和顺序直接套到本单臂接口。

## 1. 调用与响应

WebSocket 默认 `ws://<机器人地址>:9090`。服务名 `/agv/arm_move`，类型 `agv_bridge_v2_interfaces/srv/ArmMove`。

```text
string arm
float64[] joints
float64 duration_sec
---
bool success
string message
```

```json
{
  "op": "call_service",
  "id": "arm-001",
  "service": "/agv/arm_move",
  "args": {
    "arm": "left",
    "joints": [2.65, -2.6, 0, -1.62079632679, -1.57079632679, 0],
    "duration_sec": 6.0
  }
}
```

上例为回到仿真配置的初始运输姿态，会产生实际运动；联调前确认运动空间。数值是**绝对关节目标角，单位弧度**，不是角度制、增量、速度或 XYZ。

```json
{
  "op": "service_response",
  "id": "arm-001",
  "service": "/agv/arm_move",
  "result": true,
  "values": {"success": true, "message": "accepted"}
}
```

用 `id` 关联请求和响应；必须检查 `values.success`。`result:true` 不等于业务成功，`accepted` 只表示参数和执行条件通过、轨迹已发布，不表示到位，也没有执行完成事件或控制器执行成功保证。

## 2. 单臂选择与六关节顺序

- `left` 和 `right` 均控制同一只物理 UR5e；不要从两侧同时发送竞争指令。
- `waist` 拒绝：`waist not equipped`；其他名字拒绝：`invalid arm`。
- 推荐仿真直接发送 6 个角。顺序固定为**关节名字典序**，不是肩→肘的机构顺序，不是 `/joint_states` 原始数组位置。

| 六轴数组下标 | 关节名 | 机构角色 | 初始角（rad） |
|---:|---|---|---:|
| 0 | `ur5e_elbow_joint` | 肘 | 2.65 |
| 1 | `ur5e_shoulder_lift_joint` | 肩抬升 | -2.6 |
| 2 | `ur5e_shoulder_pan_joint` | 肩回转 | 0 |
| 3 | `ur5e_wrist_1_joint` | 腕1 | -1.62079632679 |
| 4 | `ur5e_wrist_2_joint` | 腕2 | -1.57079632679 |
| 5 | `ur5e_wrist_3_joint` | 腕3 | 0 |

初始角的配置来源：`simulated_ur5e/config/initial_positions.yaml`。它是仿真运输姿态，不是六轴全零，也不代表真机机械零位。上位机若按角色存储归位值 `[pan,lift,elbow,wrist1,wrist2,wrist3]`，应存 `[0,-2.6,2.65,-1.62079632679,-1.57079632679,0]`，发送前再按上表转换。归位按钮与移动联锁必须使用同一份目标；浏览器自定义归位覆盖值也需同步。

### 真机七轴请求的仿真兼容

允许 7 个角，输入约定为真机 `joint_1..joint_7`；当前默认映射配置 `real_arm_joint_indices: [3,1,0,4,5,6]`，这些是零起始输入下标，依次生成上表六轴目标：

| 输入 | 仿真目标 |
|---|---|
| joint_1 | shoulder_pan |
| joint_2 | shoulder_lift |
| joint_3 | 不仿真 |
| joint_4 | elbow |
| joint_5 | wrist_1 |
| joint_6 | wrist_2 |
| joint_7 | wrist_3 |

响应会提示 `UR5e alias; joint_3 not simulated`。这只是协议映射，不是等价七轴运动学；上位机不能用真机七轴 IK 结果推断 UR5e 的实际末端 XYZ。当前不模拟额外轴、另一只臂或腰，反馈只返回真实存在的关节。自定义 YAML 映射会改变上述对应关系。

## 3. duration_sec：必须透传的运动时长

**机器人 rosbridge 参数名必须是 `duration_sec`。** 不要在 ROS args 中写 `durationSec`。

- 单位秒，表示从当前实际关节角到目标的期望执行时长。
- 省略时 ROS 请求字段默认为 0；当前适配器对 <=0 使用 3 秒。
- 正值限制在 0.5～300 秒；0.08 秒当前也会变成至少 0.5 秒，不是立即停止。
- 目标按 URDF 位置限位截断；如果时间短于关节速度上限允许的时间，会自动延长。若所需时间超过 300 秒，拒绝执行。
- 六个关节同时插值。重复相同绝对目标不会继续累积运动。

上位机 Java HTTP API 若存在全局 Jackson `SNAKE_CASE`，推荐端到端统一：

```json
{"arm":"left","joints":[2.65,-2.6,0,-1.62079632679,-1.57079632679,0],"duration_sec":6.0}
```

当前 nav-deployer 后端已增加对旧 HTTP `durationSec` 的兼容；这是**上位机 HTTP 层**兼容，不是机器人 ROS 服务的别名。部署新后端、刷新前端后，务必检查最终 rosbridge **发送日志**确实含 `args.duration_sec`；只检查网页输入值不足以证明透传成功。

## 4. 连续点动与笛卡尔控制边界

本接口是可被新目标覆盖的关节位置轨迹接口，**不是连续速度 Servo 接口**。执行中收到新指令时，当前实现从最新关节反馈重新发布起点和新终点，不拼接轨迹，不保持跨指令速度/加速度连续性。

### 关节位置点动

上位机计算 `目标角 = 当前实际角 + 步长`，按正确关节名发送，并显式填写时长。不要把步长直接当绝对角发送；不要在延迟反馈上无限累加而不限制目标超前量。

### XYZ 笛卡尔按钮

服务本身不接收 XYZ，不做 IK、直线轨迹规划或姿态保持。上位机需先基于实际仿真 URDF、TCP 和关节限位求解，再发送六轴关节目标。关节线性插值不能保证末端严格沿 XYZ 直线，也不保证末端朝向保持不变。

当前网页的分段策略是每约 400ms 发送一次、时长 1.2s；这是上位机策略示例，不是 bridge 的固定协议要求，不构成匀速保证。曾经遗漏 `duration_sec`，导致每 400ms 把运动重置为默认 3 秒，表现为缓慢微动。

上位机实现应做到：

1. 发送前确认反馈新鲜；计算使用关节名映射，不使用混合数组的前六个位置。
2. 串行或限制在途请求；失败、断连、逆解失败或反馈过期时停止继续生成目标，不能无限堆积开环目标。
3. 明确重复周期、目标超前距离和运动时长；以真实 TCP/关节反馈测量运动效果，不能用 accepted 计数证明运动完成。
4. 逆解应在独立模型或临时姿态上计算，结束后恢复显示模型；网页模型只由真实反馈驱动，不能把求解结果当实际姿态。
5. 松开、失焦、切换页面时取消发送定时器。发“最新关节角作为新目标”只能覆盖旧轨迹，是基于反馈的近似停止；存在反馈延迟且有 0.5s 最小时长，不能作为硬急停或确定性的立即停止。

如果需要稳定持续的笛卡尔速度点动、方向/速度连续和专用停止语义，应另行对接机器人侧成熟的 Servo/笛卡尔控制能力。本接口当前不承诺这些能力，不应靠不断缩短 duration 掩盖缺失功能。

## 5. 实际关节反馈与到位判定

通过 rosbridge 订阅：

```json
{"op":"subscribe","id":"arm-feedback","topic":"/joint_states","type":"sensor_msgs/msg/JointState","throttle_rate":100}
```

消息为标准 `sensor_msgs/msg/JointState`：`name[i]` 与 `position[i]` 一一对应。数组包含底盘与六个 UR5e 关节，顺序可能交错；按名称提取上表六个，不能假定前六个是机械臂。角度单位 rad，速度单位 rad/s；Gazebo 部分 effort 未提供，rosbridge 可能编码为 null，不能据此丢弃整条有效 position。

只发布实际存在的关节，不为左右臂或七轴补造数据。网页 URDF 应以完全相同的关节名调用 `setJointValue`；关节数据不依赖 `/agv/arm_move` 返回值。

到位判定由上位机实现：持续检查所有目标关节的位置误差、可用的速度和反馈新鲜度，连续多帧满足容差才判完成；容差按业务配置。本服务尚无完成 action、取消接口或执行中错误回调。若目标因限位被截断，上位机必须结合 URDF 限位判断，不能永远等待原始超限目标。仿真消息时间戳使用仿真时间，不要直接与系统墙钟相减。

## 6. 拒绝条件与错误

| message | 含义 |
|---|---|
| `invalid arm` | 不是 left/right/waist |
| `waist not equipped` | 单臂仿真没有腰 |
| `expected 6 or 7 joints` | 数量不符 |
| `joints and duration_sec must be finite` | NaN/Inf 等无效数值 |
| `joint states unavailable or stale` | 六个关节未齐或反馈接收超过 2 秒 |
| `joint_trajectory_controller not ready` | 控制器未确认 active、确认过期或无轨迹订阅端 |
| `motion requires more than 300 seconds within joint velocity limits` | 满足限速所需时间太长 |

收到拒绝不要显示“运动成功”；记录 service id、目标、时长、响应以及实际反馈。`accepted` 后控制器是否成功执行仍需通过反馈确认。

## 7. 联调验收清单

1. 启动整机仿真 + 整机 bridge，确认服务存在、控制器 active、六个关节数据新鲜。
2. 发单次已知目标，核对 ROS args 包含 `duration_sec`、关节名顺序、弧度单位；确认响应 accepted。
3. 在预期时间内检查实际六关节反馈接近目标，网页 3D URDF 同步变化；不要仅观察调试 IK 模型。
4. 按住 XYZ 按钮时同时记录下发目标、时长、关节反馈和实际 TCP；验证方向/位移。松开后确认没有残留发送，不把近似冻结当硬急停。
5. 断连、无关节反馈、控制器未就绪、逆解失败时停止生成后续目标并显示明确错误。

本次已验证时长字段两种 HTTP 命名的反序列化、前端构建、IK 成功/失败后显示姿态恢复；尚未验证真实连续 XYZ 运动的速度、路径精度和停止距离。本文不将这些列为已实现的 Servo 能力。
