# 通用双指仿真夹爪

UR5e 组合机器人默认使用 `tool_type:=gripper`；`camera` 保留原腕部相机，
`none` 只保留法兰。仿真与 bridge 必须选择相同工具和配置文件。

## 模型和启动

圆柱底座与两个平行手指均使用 URDF 基本几何体，无外部 mesh。
两个独立 prismatic 关节沿局部 Y 轴反向移动；最大净开口 80 mm、初始 60 mm，
每指最大速度 40 mm/s。模型包含质量、惯量、碰撞体及手指摩擦参数。
新增 `gripper_controller` 与原机械臂 JTC 共用一个整机 controller_manager，
不修改原六轴 `/agv/arm_move` 接口。实际手指状态加入 `/joint_states`。

夹持 TCP 为 `ur5e_gripper_tcp`，默认距 tool0 局部 +Z 125 mm
（安装偏移 15 mm + 夹持中心 110 mm）。MoveIt 机械臂链和 arm_manager 位姿目标
切换到该 TCP；夹爪独立组 `ur5e_gripper` 有 `open`、`closed` 姿态。
旧的基于 tool0 的位姿示教点不能直接当作夹持中心位姿，切换工具后须重新核对。

构建并加载工作空间环境后，在两个终端分别运行：

```bash
ros2 launch simulated_chassis_ur5e simulation.launch.py tool_type:=gripper
ros2 launch simulated_chassis_ur5e bridge.launch.py tool_type:=gripper
```

切回原相机时先停止两端进程，再将两边都改为 `tool_type:=camera`。
gripper/none 模式不生成相机，也不启动图像桥接和拍照节点。

Docker 两端共用 `.env` 的 `TOOL_TYPE=gripper`，可改为 `camera` 或 `none`。
首次安装功能需重建业务镜像和容器：

```bash
docker compose build robot-simulation agv-bridge
bash deploy.sh restart
```

仅切换已内置工具不需要重编译镜像，但仍需重建两容器。
`config/gripper.yaml` 提供最大/初始开口、速度、动作时间和安装位姿。
长度单位米，角度弧度。当前几何体支持最大开口 `(0, 0.08]` m；
修改爪体尺寸需同步调整惯量与 TCP。自定义 `gripper_file` 需同时传给两端；
Docker 包装脚本默认使用镜像内文件，不直接挂载外部夹爪配置。

## 接口

| 接口 | 类型 | 含义 |
|---|---|---|
| `/gripper/set_closed` | `std_srvs/srv/SetBool` | `data=true` 闭合，`false` 完全打开 |
| `/gripper/width` | `std_msgs/msg/Float64` | 实际两指内侧净开口，米；从关节反馈计算 |
| `/gripper/status` | `std_msgs/msg/String` | 运行状态，transient-local 保留最近状态 |
| `/joint_states` | `sensor_msgs/msg/JointState` | 两指关节名 `ur5e_gripper_left_joint`、`ur5e_gripper_right_joint`；位置为单指行程 |

```bash
ros2 service call /gripper/set_closed std_srvs/srv/SetBool '{data: false}'
ros2 service call /gripper/set_closed std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /gripper/width
ros2 topic echo /gripper/status --qos-durability transient_local
```

UR5e bridge 的 rosbridge 白名单已放行服务和状态话题，无新增端口，复用 9090。
WebSocket 请求：

```json
{"op":"call_service","id":"gripper-close-1","service":"/gripper/set_closed","args":{"data":true}}
```

服务成功只表示已提交给控制器 Action 客户端，不表示控制器已接受或夹住物体。
状态包括 `waiting`、`idle`、`moving`、`succeeded`、`rejected`、`aborted`、
`timeout`、`state_stale`。执行期间拒绝重复服务请求，关节缺失、非有限或超两秒
未更新时拒绝新命令。执行中状态失联或墙钟超时会尝试取消本节点的目标。

任意开口可以通过 ROS 原生 `/gripper_controller/follow_joint_trajectory` Action 控制，
两个目标位置均为“所需净开口 / 2”。直接 Action/MoveIt 与开合服务共享执行器，
没有全局任务仲裁，请勿并发下发。

本实现提供物理关节和可控开合，`succeeded` 表示控制器轨迹成功，不检测抓持力、
接触或物体，也不创建吸附/固定约束。稳定夹持并抬起物体需要另做物理场景验证。

## 轻量验证

`test_tool_models.py` 检查工具互斥、关节几何/行程、控制器和 MoveIt/TCP 配置，
并通过 MoveIt/FCL 检查折叠姿态下闭合、初始和最大开口的夹爪相关自碰撞。
`test_gripper_control.py` 使用独立 ROS domain 187 和假控制器，验证 C++ 服务的
开合目标、实际宽度、速度限制、忙碌、拒绝、失败、过期/非有限状态。
这些测试不启动 Gazebo，不移动真实或现有仿真机器人。

```bash
colcon test --packages-select simulated_chassis_ur5e --event-handlers console_direct+
colcon test-result --verbose
```
