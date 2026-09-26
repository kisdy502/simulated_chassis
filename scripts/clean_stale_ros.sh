#!/usr/bin/env bash
# 清理遗留的 localization/slam launch 会话（僵尸 wrapper：父进程已死、被 systemd 托管）。
# 场景：RViz 地图闪烁（多个 occupancy_grid_node 同发 /map）、重启整套仿真前。
# 原理：桥接正常 Ctrl+C 会优雅停掉自己拉起的定位/建图子进程；
#       但终端直接关闭/强杀时子进程（独立进程组）会变孤儿存活下来。
set -u

count=0
for pid in $(pgrep -f "ros2 launch .*localization\.launch\.py|ros2 launch .*slam\.launch\.py"); do
    [ "$pid" = "$$" ] && continue
    ppid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$ppid" ] && continue
    pcmd=$(ps -o cmd= -p "$ppid" 2>/dev/null)
    if [[ "$pcmd" == *systemd* ]]; then
        age=$(ps -o etime= -p "$pid" | tr -d ' ')
        echo "僵尸 launch wrapper: pid=$pid 已存活 $age → SIGINT"
        kill -INT "$pid"
        count=$((count + 1))
    fi
done

if [ "$count" -eq 0 ]; then
    echo "无僵尸 launch 会话"
else
    sleep 3
    echo "剩余 occupancy_grid_node:"
    pgrep -af "occupancy_grid_node" | grep -v pgrep || echo "  （无）"
fi
