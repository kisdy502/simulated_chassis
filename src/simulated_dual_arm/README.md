# simulated_dual_arm —— 双 7 轴臂 + 2 轴腰上装仿真(F1)

线束质检机器人整机仿真入口之一:三舵轮底盘(复用 `simulated_chassis` 宏)+ 视觉级双臂上装。
满足上位机(nav-deployer)机械臂接入契约,详细需求见 nav-deployer《仿真机器人功能开发说明》。

## 使用

```bash
colcon build --symlink-install --packages-up-to simulated_dual_arm
source install/setup.bash

# 终端一:物理仿真(手柄遥控加 start_gamepad:=true)
ros2 launch simulated_dual_arm simulation.launch.py gui:=true

# 终端二:bridge(含上装节点与 mesh 服务)
ros2 launch simulated_dual_arm bridge.launch.py slam_algorithm:=lightning
```

## F1 契约对应

| 契约项 | 实现 |
|---|---|
| `/joint_states` 合并(舵轮 steer + arm_left/right_1..7 + waist_1..2) | `dual_arm_sim_node` 20Hz,自过滤防回环 |
| `robot_description` 可读 | `/robot_state_publisher/*` 已加 params_glob |
| mesh 静态服务 | `urdf_mesh_server.py` 默认 8090,`/<pkg>/<相对路径>` → ament share;前端 `VITE_URDF_MESH_BASE=http://<host>:8090` |
| `/agv/arm_move` | `agv_bridge_v2_interfaces/srv/ArmMove`;s 型插值执行,同组忙时拒绝;不动 `/agv/status.mode` |
| 模型 | 几何基元(暂无图纸);安装位 x=-0.1, y=±0.25, z=0.15;Gazebo 高阻尼防下垂、无动力学 |

## 备注

- 上装关节仅视觉仿真(不接 ros2_control);真实 mesh 到位后替换 `urdf/dual_arm_macro.xacro` 中的 `<geometry>` 并保持 joint 命名不变;
- `AgvStatus` 已扩展 `slam_alive` / `map_age_sec`(F2,上位机仅提示用);
- 与 `simulated_chassis_ur5e`(UR5e 对比链路)互不影响,同一时间只能跑一个整机 profile。
