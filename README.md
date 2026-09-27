# 仿真机器人+agv_bridge节点

## 启动顺序

### 1仿真机器人

source install/setup.bash
ros2 launch simulated_chassis three_wheel_sim.launch.py

默认不弹 xterm 键盘遥控窗口（launch 退出杀不掉它，是进程残留大户；日常遥控用手柄）。
需要键盘遥控时：

ros2 launch simulated_chassis three_wheel_sim.launch.py start_teleop:=true

### 2导航

ros2 launch simulated_chassis navigation.launch.py include_localization:=false

### 3桥接节点
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py  robot_package:=simulated_chassis

## 重启前清场（重要）

直接关终端窗口 / 异常退出会留孤儿进程（xterm 遥控、rosapi、ros2 daemon 等），
它们继续持有 FastDDS 共享内存端口，下次启动概率性失败（init_port 报错、
`ros2 node list` 出现幻影节点）。整套重启前先跑：

```bash
scripts/clean_all_ros.sh
```

日常退出请尽量用 Ctrl+C（让 ros2 launch 优雅回收子节点），别直接关终端窗口。

