#!/bin/bash

# ============================================================
# ROS 2 清理 + 启动脚本
# 用法: ./ros2_clean_launch.sh <launch包名> <launch文件名>
# 示例: ./ros2_clean_launch.sh agv_nav agv_nav.launch.py
# ============================================================

# ---------- 可配置区域 ----------
# 你要启动的 launch（也可以通过命令行参数传入）
LAUNCH_PKG="${1:-your_pkg}"
LAUNCH_FILE="${2:-your_launch.launch.py}"

# 清理等待时间（秒），给 DDS 一点时间回收
WAIT_SEC=2
# --------------------------------

echo "=========================================="
echo "[1/4] 清理前 ROS 2 节点列表:"
echo "=========================================="
ros2 node list 2>/dev/null || echo "(ros2 node list 不可用)"

echo ""
echo "=========================================="
echo "[2/4] 开始清理残留 ROS 2 进程..."
echo "=========================================="

# 按常见 ROS 2 可执行/进程特征批量杀
# 注意: 不会杀 ros2 daemon，避免影响后续命令
pkill -f "ros2 launch"        2>/dev/null
pkill -f "ros2 run"           2>/dev/null
pkill -f "component_container" 2>/dev/null
pkill -f "nav2_"              2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "rviz2"              2>/dev/null
pkill -f "rosbridge_websocket" 2>/dev/null
pkill -f "agv_nav_server"     2>/dev/null
pkill -f "slam_toolbox"       2>/dev/null
pkill -f "controller_server"  2>/dev/null
pkill -f "planner_server"     2>/dev/null
pkill -f "bt_navigator"       2>/dev/null
pkill -f "behavior_server"    2>/dev/null
pkill -f "velocity_smoother"  2>/dev/null
pkill -f "waypoint_follower"  2>/dev/null
pkill -f "lifecycle_manager"  2>/dev/null

# 如果你有自定义节点，按名字补充，例如:
# pkill -f "my_custom_node"
pkill -f "/home/kisdy/workspace/simulated_chassis/install"

echo "已发送终止信号，等待 ${WAIT_SEC}s 让进程退出..."
sleep "$WAIT_SEC"

# 如果还有顽固进程，强制杀
echo "检查是否还有残留..."
REMAIN=$(ps aux | grep -E "ros2 launch|ros2 run|component_container|nav2_|agv_nav_server|controller_server|planner_server|bt_navigator|behavior_server|velocity_smoother|waypoint_follower|lifecycle_manager" | grep -v grep)

if [ -n "$REMAIN" ]; then
    echo "仍有残留，执行强制清理:"
    echo "$REMAIN"
    pkill -9 -f "ros2 launch"        2>/dev/null
    pkill -9 -f "ros2 run"           2>/dev/null
    pkill -9 -f "component_container" 2>/dev/null
    pkill -9 -f "nav2_"              2>/dev/null
    pkill -9 -f "agv_nav_server"     2>/dev/null
    pkill -9 -f "controller_server"  2>/dev/null
    pkill -9 -f "planner_server"     2>/dev/null
    pkill -9 -f "bt_navigator"       2>/dev/null
    pkill -9 -f "behavior_server"    2>/dev/null
    pkill -9 -f "velocity_smoother"  2>/dev/null
    pkill -9 -f "waypoint_follower"  2>/dev/null
    pkill -9 -f "lifecycle_manager"  2>/dev/null
    sleep 1
else
    echo "无残留进程。"
fi

echo ""
echo "=========================================="
echo "[3/4] 清理后 ROS 2 节点列表:"
echo "=========================================="
ros2 node list 2>/dev/null || echo "(ros2 node list 不可用)"

echo ""
echo "=========================================="
echo "[4/4] 启动 launch: ${LAUNCH_PKG} ${LAUNCH_FILE}"
echo "=========================================="
source /opt/ros/humble/setup.bash
# 如果你有工作空间，也要 source:
# source ~/your_ws/install/setup.bash

ros2 launch "$LAUNCH_PKG" "$LAUNCH_FILE"