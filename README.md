# 仿真机器人+agv_bridge节点

Docker 镜像构建、双容器部署、三维地图/URDF 资源服务、机械臂与相机接入见
[Docker 部署说明](DOCKER_DEPLOY.md)。

UR5e 组合机器人默认末端为通用双指夹爪，支持开合与实际状态反馈。
两端设置 `tool_type:=camera`（Docker 为 `TOOL_TYPE=camera`）可切回原相机，
使用方法见 [夹爪说明](docs/gripper_api.md)。

## 建图 / 定位 / 导航（bridge 统一托管）

仿真单独启动，其余节点只启动 bridge 一个入口，由它托管定位/建图/Nav2 并支持
`slam_algorithm:=cartographer|lightning` 后端切换。完整步骤见
[docs/mapping_and_localization.md](docs/mapping_and_localization.md)。

## 三舵轮 3D 雷达＋UR5e 上装（新组合入口）

上装的模型、安装参数、控制/规划配置集中在 `src/simulated_chassis_ur5e`；通用 bridge 和原有纯底盘入口保留。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select battery_simulator arm_manager_interfaces arm_manager simulated_chassis simulated_ur5e simulated_chassis_ur5e
source install/setup.bash
# 终端一：整机物理仿真（需要图形显示）
ros2 launch simulated_chassis_ur5e simulation.launch.py gui:=true
# 终端二：bridge 自动带起机械臂运行节点
ros2 launch simulated_chassis_ur5e bridge.launch.py rviz:=true
```

无需另外启动 `simulated_ur5e` 或 `arm_manager`。机器人直接以折叠姿态生成；bridge 重启不重置姿态。无界面和完整配置见 [组合包说明](src/simulated_chassis_ur5e/README.md)。

Docker 默认 `ROBOT_PROFILE=chassis_3d_ur5e`；需要原纯底盘时设为 `chassis_3d`。下面保留原纯底盘手动调试流程。

## 原纯底盘启动顺序（不适用于当前 UR5e 整机）

当前带机械臂的机器人应使用上面的 `simulated_chassis_ur5e` 组合入口。下面通用 `agv_rosbridge.launch.py` 不启动机械臂运行节点和8090 mesh服务，不能替代组合 bridge。

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
source ~/.bashrc   # 或直接 export SIM_WS_ROOT=/home/kisdy
ros2 launch simulated_chassis_ur5e bridge.launch.py \
  slam_algorithm:=lightning \
  maps_dir:=/home/kisdy/maps \
  pbstream_file:=/home/kisdy/maps/map1025L
    

### 当前 UR5e 整机加载 Lightning 地图（推荐）

已有整机仿真保持运行；先在旧 bridge 终端按 Ctrl+C，等待退出，再在加载过工作区环境的终端启动下面这一条。不要同时运行通用 bridge 和组合 bridge，也不要重复启动仿真。

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch simulated_chassis_ur5e bridge.launch.py \
  slam_algorithm:=lightning \
  maps_dir:=/home/kisdy/maps \
  pbstream_file:=/home/kisdy/maps/map1025L
```

组合 bridge 包含通用导航桥接、机械臂运行节点以及8090 mesh服务；无需单独启动 `urdf_mesh_server`。完整配置以 [组合包启动说明](src/simulated_chassis_ur5e/README.md#启动) 为准。

## 重启前清场（重要）

直接关终端窗口 / 异常退出会留孤儿进程（xterm 遥控、rosapi、ros2 daemon 等），
它们继续持有 FastDDS 共享内存端口，下次启动概率性失败（init_port 报错、
`ros2 node list` 出现幻影节点）。整套重启前先跑：

```bash
scripts/clean_all_ros.sh
```

日常退出请尽量用 Ctrl+C（让 ros2 launch 优雅回收子节点），别直接关终端窗口。

## 禁止全量删除，只清理指定包；保留 log

日常修改优先直接增量编译，无需先删除。下面每组命令仅在需要清理对应包时单独复制执行，不要整段执行。

## battery_simulator：C++ 电量模拟节点

```bash
rm -rf -- build/battery_simulator install/battery_simulator
colcon build --packages-select battery_simulator
source install/setup.bash
```

仿真入口默认启动电池节点；首次使用新版本前先编译此包。电量持久化文件不在 build/install 内，清理编译产物不会重置电量。

## agv_bridge_v2：导航 bridge
rm -rf -- build/agv_bridge_v2 install/agv_bridge_v2
colcon build --packages-select agv_bridge_v2

## agv_bridge_v2_interfaces：导航接口
rm -rf -- build/agv_bridge_v2_interfaces install/agv_bridge_v2_interfaces
colcon build --packages-select agv_bridge_v2_interfaces

## agv_map_3d：3D 地图服务
rm -rf -- build/agv_map_3d install/agv_map_3d
colcon build --packages-select agv_map_3d

## simulated_chassis：仿真底盘
rm -rf -- build/simulated_chassis install/simulated_chassis
colcon build --packages-select simulated_chassis

## simulated_chassis_ur5e：底盘与 UR5e 组合机器人
rm -rf -- build/simulated_chassis_ur5e install/simulated_chassis_ur5e
colcon build --packages-select simulated_chassis_ur5e

## simulated_ur5e：UR5e 仿真
rm -rf -- build/simulated_ur5e install/simulated_ur5e
colcon build --packages-select simulated_ur5e

## simulated_arm：机械臂仿真
rm -rf -- build/simulated_arm install/simulated_arm
colcon build --packages-select simulated_arm

## simulated_dual_arm：双臂仿真
rm -rf -- build/simulated_dual_arm install/simulated_dual_arm
colcon build --packages-select simulated_dual_arm

## arm_manager：机械臂任务管理
rm -rf -- build/arm_manager install/arm_manager
colcon build --packages-select arm_manager

## arm_manager_interfaces：机械臂接口
rm -rf -- build/arm_manager_interfaces install/arm_manager_interfaces
colcon build --packages-select arm_manager_interfaces

## jzt_robot
rm -rf -- build/jzt_robot install/jzt_robot
colcon build --packages-select jzt_robot

## zioneer_robot
rm -rf -- build/zioneer_robot install/zioneer_robot
colcon build --packages-select zioneer_robot



## 如果修改了导航接口的消息、服务或 action 字段结构，应同时重建当前直接依赖它的包
colcon build --packages-select \
  agv_bridge_v2_interfaces \
  agv_bridge_v2 \
  simulated_chassis_ur5e \
  simulated_dual_arm

## 首次接入电量/充电功能：按包联动编译，不删除已有安装

本次新增 battery_simulator，并扩展 AgvStatus 消息；仅编译电池包不能更新 bridge 的充电接口和状态字段。

```bash
source /opt/ros/humble/setup.bash
colcon build --executor sequential --packages-select \
  battery_simulator \
  agv_bridge_v2_interfaces \
  agv_bridge_v2 \
  simulated_chassis \
  simulated_chassis_ur5e \
  simulated_dual_arm
source install/setup.bash
```

上面按已有其他依赖已安装编写，不包含 Lightning，也不会执行全量构建。
Docker 部署需同步更新机器人与 bridge 镜像；接口说明见 [电量与充电对接](docs/battery_api.md)。

## Lightning 单独构建
bash scripts/build_lightning.sh
source install/setup.bash


完整接口和专题文档见 [文档索引](docs/README.md)。
