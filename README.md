# 仿真机器人+agv_bridge节点

## 建图 / 定位 / 导航（bridge 统一托管）

仿真单独启动，其余节点只启动 bridge 一个入口，由它托管定位/建图/Nav2 并支持
`slam_algorithm:=cartographer|lightning` 后端切换。完整步骤见
[docs/mapping_and_localization.md](docs/mapping_and_localization.md)。

## 三舵轮 3D 雷达＋UR5e 上装（新组合入口）

上装的模型、安装参数、控制/规划配置集中在 `src/simulated_chassis_ur5e`；通用 bridge 和原有纯底盘入口保留。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select arm_manager_interfaces arm_manager simulated_chassis simulated_ur5e simulated_chassis_ur5e
source install/setup.bash
# 终端一：整机物理仿真（需要图形显示）
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true
# 终端二：bridge 自动带起机械臂运行节点
ros2 launch simulated_chassis_ur5e bridge.launch.py rviz:=true
```

无需另外启动 `simulated_ur5e` 或 `arm_manager`。机器人直接以折叠姿态生成；bridge 重启不重置姿态。无界面和完整配置见 [组合包说明](src/simulated_chassis_ur5e/README.md)。

Docker 默认 `ROBOT_PROFILE=chassis_3d_ur5e`；需要原纯底盘时设为 `chassis_3d`。下面保留原纯底盘手动调试流程。

## 启动顺序

### 1仿真机器人

source install/setup.bash
ros2 launch simulated_chassis three_wheel_sim.launch.py

默认不弹 xterm 键盘遥控窗口（launch 退出杀不掉它，是进程残留大户；日常遥控用手柄）。
需要键盘遥控时：

ros2 launch simulated_chassis three_wheel_sim.launch.py start_teleop:=true

### 2导航（不用启动了，交给bridge启动）

ros2 launch simulated_chassis navigation.launch.py include_localization:=false

### 3桥接节点
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py  robot_package:=simulated_chassis

### 有地图加载，
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
        pbstream_file:=$PWD/maps/map1008Pro/map1008Pro.pbstream \
        robot_package:=simulated_chassis

## 重启前清场（重要）

直接关终端窗口 / 异常退出会留孤儿进程（xterm 遥控、rosapi、ros2 daemon 等），
它们继续持有 FastDDS 共享内存端口，下次启动概率性失败（init_port 报错、
`ros2 node list` 出现幻影节点）。整套重启前先跑：

```bash
scripts/clean_all_ros.sh
```

日常退出请尽量用 Ctrl+C（让 ros2 launch 优雅回收子节点），别直接关终端窗口。

