# 三舵轮 3D 雷达 + UR5e 整机仿真

本包只负责仿真组合：安装座、带 `ur5e_` 前缀的机械臂、组合碰撞/控制配置、启动编排。底盘算法继续在 `simulated_chassis`，机械臂业务在 `arm_manager`，单臂调试在 `simulated_ur5e`，通用/真机 bridge 保持独立。

## 上层真机协议与单臂仿真映射

`/agv/arm_move` 由 C++ `arm_move_adapter` 提供，服务类型仍为
`agv_bridge_v2_interfaces/srv/ArmMove`，JSON/IDL 字段不变。`left` 和 `right`
均控制同一个 UR5e，后来的请求从最新实测角度重新生成轨迹并替换前一个目标。
它们不代表仿真中存在两条独立机械臂。

上层可以继续发送真机每臂 7 个角度（joint_1..joint_7）。默认映射如下：

| 输入轴（从 1 开始） | UR5e 实体关节 |
|---|---|
| 1 | shoulder_pan |
| 2 | shoulder_lift |
| 3 | 额外上臂回转不仿真，响应明确包含 `joint_3 not simulated` |
| 4 | elbow |
| 5 | wrist_1 |
| 6 | wrist_2 |
| 7 | wrist_3，保留末端回转 |

这是为上层协议联调提供的单臂别名映射，不是 7 轴机械臂的等效运动学。
索引配置见 `config/arm_move_adapter.yaml` 的 `real_arm_joint_indices`，
它在 UR5e 字典序中使用零起始索引，默认 `[3,1,0,4,5,6]`。
如改变配置，未被映射的输入轴会在响应中按实际索引提示。

兼容原 6 轴请求，其顺序为 UR5e **字典序**：
`elbow, shoulder_lift, shoulder_pan, wrist_1, wrist_2, wrist_3`。
字典序与机构顺序不同，调用端不可混用。数量为 6/7 以外或包含 NaN/Inf 时拒绝。
腰部未装备，`arm=waist`（即使给出 2 个角度）返回
`success=false, message="waist not equipped"`，不伪造腰部动作完成。

回调只做校验并下发两点 `JointTrajectory`，不会等待几秒运动结束。
JTC 在其控制周期内做线性插值，不需要适配节点再以 50Hz 发布角度。
`duration_sec<=0` 使用 3 秒；正时长限制为 0.5～300 秒。角度按当前整机 URDF
限位截断，必要时延长时间，使所生成线性参考轨迹不超过 URDF 关节速度限值。
这不提供加速度约束、MoveIt 避碰规划或物理到位保证。

接受请求前须收到 2 秒内完整、有限的实测 6 轴状态，并且控制器状态查询显示
`joint_trajectory_controller` 为 active、查询结果未过期、轨迹话题有订阅者；
未就绪/失联返回 false，不能用历史状态继续下发。
`success=true` 表示适配层已受理并发布，不表示控制器确认完成。
执行情况通过 JTC 日志和实际 `/joint_states` 观察。

上报保持实际 URDF 的 `ur5e_*` 六关节（以及原底盘关节），不额外发布虚构的
左臂、右臂或腰关节；不修改 ArmMove 响应去拼装虚拟状态。rosbridge 已放行
`/agv/arm_move` 和 `/joint_states`，上位机通过原链路驱动当前 URDF 模型。

```bash
# 7轴输入跳过额外上臂回转轴3，保留末端轴7
ros2 service call /agv/arm_move agv_bridge_v2_interfaces/srv/ArmMove \
  "{arm: left, joints: [0.5, -1.2, 0.0, 1.0, -0.5, 0.0, 0.0], duration_sec: 4.0}"

# 只有实际 UR5e 和底盘状态，不虚构 16 轴上装
ros2 topic echo /joint_states
```

隔离契约测试（ROS domain 88，假控制器，不移动机器人）：

```bash
colcon test --packages-select simulated_chassis_ur5e --ctest-args -R test_arm_move_adapter --output-on-failure
colcon test-result --verbose
```

## 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select agv_bridge_v2_interfaces agv_bridge_v2 arm_manager_interfaces arm_manager simulated_chassis simulated_ur5e simulated_chassis_ur5e
source install/setup.bash

# 终端一：一个世界、一台组合机器人、一个 controller_manager
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true

# 终端二：原有 bridge + MoveIt + arm_manager，无需单独启动机械臂
ros2 launch simulated_chassis_ur5e bridge.launch.py rviz:=true

# 如需加载已有地图，用下面命令替代上一条 bridge 命令；定位就绪后启动托管导航
ros2 launch simulated_chassis_ur5e bridge.launch.py \
  slam_algorithm:=lightning \
  maps_dir:=$PWD/maps \
  pbstream_file:=$PWD/maps/map1020
```

无界面 GPU 雷达仍需渲染上下文，可用 `xvfb-run -a ros2 launch simulated_chassis_ur5e simulation.launch.py`；Docker 无界面入口已使用 xvfb。

bridge 入口支持 `agv_id`、`port`、`address`、`maps_dir`、`pbstream_file`、`manage_navigation` 等原有配置。`rviz` 默认 false，`allow_fault_injection` 默认 false。启动 bridge 前会检查实际 robot_description、全部控制器和机械臂关节状态就绪；不匹配或重复启动运行节点时拒绝启动机械臂。bridge 重启不会重新生成模型或执行收臂动作。

整机入口只保留机械臂 RViz（`rviz_mobile_ur5e`），托管导航显式关闭自己的 RViz。该窗口同时包含整机模型、MoveIt、地图和导航路径显示；定位后可将 Fixed Frame 改为 `map` 查看全局地图。

在 bridge 所在终端按 Ctrl+C，会同步退出其启动的定位、建图、导航、MoveIt、arm_manager 和 RViz。等待终端返回命令提示符后再重启；退出时先发送中断信号，超时再逐级终止进程组。核心 bridge 节点异常退出也会触发外层 launch 关闭。独立终端启动的仿真仍由仿真终端管理，关闭 RViz 窗口本身不会关闭整套 bridge。

## 配置归属

| 文件 | 用途 |
|---|---|
| `urdf/robot.urdf.xacro` | 底盘宏 + 上装宏 |
| `urdf/arm_mount.xacro` | 安装座惯量/碰撞与官方 UR5e 宏调用 |
| `config/mounting.yaml` | 安装座半径、高度、质量、相对车体位置 |
| `simulated_ur5e/config/initial_positions.yaml` | 单臂、整机、SRDF 共用的折叠初始角度 |
| `config/controllers.yaml` | 仅新增机械臂控制配置，启动时与原底盘参数合并 |
| `config/moveit/robot.srdf.xacro` | 带车体/雷达/安装座的组合碰撞模型和 stowed 位姿 |

安装座默认从车体顶面 z=0.09 m 起，高 0.22 m，质量 3 kg。修改安装配置后，仿真和 bridge 均须传同一 `mounting_file`。初始姿态直接由仿真硬件初始化，不发送开机运动；这是仿真运输姿态，不代表厂商统一出厂姿态。

底盘和机械臂的两个 ros2_control 硬件组件由同一个 Ignition 插件、一个 manager 管理。只保留一个完整 joint_state_broadcaster、robot_state_publisher 和 /clock。MoveIt 只规划机械臂六轴，底盘关节为 passive；车体和雷达保留碰撞体。第一阶段不支持边行走边作业，操作机械臂时保持底盘停车。

原纯底盘的雷达只有 visual，组合 profile 单独启用与其外形一致的圆柱/球体碰撞几何，供机械臂避碰；纯底盘默认不启用，展开模型保持兼容。

## 接口及对比入口

机械臂仍使用 `/arm_manager/command`、`/arm_manager/status` 等原有业务接口，关节名称变为 `ur5e_*`，末端为 `ur5e_tool0`，规划组为 `ur5e_ur_manipulator`。组合示教点单独保存在 `~/.ros/arm_manager/mobile_ur5e_teach_points.json`，不覆盖单臂示教点。

已有 `/joint_states` 和 TF 可观察实际姿态；新的上位机姿态查询接口本阶段未增加。bridge 已允许订阅 manager 状态和调用示教/取消/软件复位服务；模拟故障服务未开放给远端。

纯底盘仍可用 `ros2 launch simulated_chassis three_wheel_sim.launch.py`，独立 UR5e 仍可用 `ros2 launch simulated_ur5e ur5e.launch.py mode:=mock`。它们与整机不能在同一个 ROS_DOMAIN_ID 同时启动。

## 验证

```bash
# 实际 Fortress：折叠初始值、控制器唯一性、整机碰撞、前进/横移/转向、机械臂执行、bridge 重启保持姿态
python3 src/simulated_chassis_ur5e/test/integration_fortress.py
# 附加 3D 建图、保存、定位和托管导航回归
python3 src/simulated_chassis_ur5e/test/integration_fortress.py --navigation
# 增加 RViz 唯一性、正常退出清理、bridge 节点异常退出清理检查（需要 xvfb-run）
python3 src/simulated_chassis_ur5e/test/integration_fortress.py --navigation --shutdown-check
# 服务等待中退出，以及启动器提前退出后遗留子进程的回收检查
python3 src/agv_bridge_v2/scripts/test_managed_shutdown.py
```

测试使用独立 ROS_DOMAIN_ID=92、IGN_PARTITION、临时地图与示教目录，不修改用户地图或示教点。Docker Compose 和入口脚本另做静态检查；镜像仍需按部署流程重建。
