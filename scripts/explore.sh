#!/bin/bash
# 简单随机探索脚本 - 用于SLAM建图
# 让机器人自动移动，覆盖周围环境

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "=========================================="
echo "开始自动探索 (按 Ctrl-C 停止)"
echo "=========================================="

# 探索参数
LINEAR_SPEED=${1:-0.3}      # 前进速度 m/s
TURN_SPEED=${2:-0.5}        # 转弯速度 rad/s
STRAIGHT_TIME=${3:-4}       # 直走时间 秒
TURN_TIME=${4:-1.5}         # 转弯时间 秒

echo "速度: 前进 ${LINEAR_SPEED} m/s, 转弯 ${TURN_SPEED} rad/s"
echo "模式: 直走${STRAIGHT_TIME}s → 转弯${TURN_TIME}s"
echo ""

# 探索模式计数器
mode=0
iteration=0

trap 'echo ""; echo "停止探索，共运行 ${iteration} 个循环"; ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once; exit' INT TERM

while true; do
    iteration=$((iteration + 1))
    
    case $mode in
        0)  # 直走
            echo "[$iteration] 直走..."
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: $LINEAR_SPEED}, angular: {z: 0.0}}" --once
            sleep $STRAIGHT_TIME
            mode=1
            ;;
        1)  # 左转
            echo "[$iteration] 左转..."
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.0}, angular: {z: $TURN_SPEED}}" --once
            sleep $TURN_TIME
            mode=2
            ;;
        2)  # 直走
            echo "[$iteration] 直走..."
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: $LINEAR_SPEED}, angular: {z: 0.0}}" --once
            sleep $STRAIGHT_TIME
            mode=3
            ;;
        3)  # 右转（反向）
            echo "[$iteration] 右转..."
            ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
                "{linear: {x: 0.0}, angular: {z: -$TURN_SPEED}}" --once
            sleep $TURN_TIME
            mode=0
            ;;
    esac
    
    # 每10个循环打印地图状态
    if [ $((iteration % 10)) -eq 0 ]; then
        map_size=$(timeout 2 ros2 topic echo /map --once 2>/dev/null | grep -A1 "width:" | tail -1)
        echo "--- 已运行 ${iteration} 次循环, 地图尺寸: $map_size ---"
    fi
done
