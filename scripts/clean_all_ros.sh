#!/usr/bin/env bash
# ============================================================
# 全量清场：停掉本机所有 ROS 相关进程 + 刷新 ros2 daemon + 清理
# /dev/shm 的 FastDDS 残留（孤儿端口/信号量是 "Failed init_port"
# 报错与重启后图紊乱的来源）。
#
# 何时用：整套重启之前；终端异常退出/直接关窗后图里出现幻影节点时。
# ⚠️ 会杀掉正在跑的整套仿真/导航/桥接/遥控——只在准备全停时执行。
#
# 与 clean_stale_ros.sh 的分工：那个只清"已判定为孤儿"的
# localization/slam launch 会话（不影响在跑的栈）；本脚本是无差别全停。
#
# 匹配范围按路径而非节点名枚举（节点名永远列不全，rosapi_node /
# teleop / xterm 包装 / cartographer / map_saver 全都吃过这个亏）：
#   /opt/ros/humble            一切发行版自带的 ROS 可执行（含
#                              "xterm -e /opt/ros/..." 的包装进程）
#   simulated_chassis/install  本工作空间编译出的节点（agv_nav_server 等）
#   ros2 launch / ros2 run     wrapper 本体
#   ros2cli / ros2-daemon      ros2 daemon（cmdline 不含 /opt/ros 路径，
#                              不杀它就清不掉它持有的 fastrtps 共享内存）
#   ign gazebo                 仿真宿主（进程名不是 gz sim）
# 模式中的方括号（如 humbl[e]）用于打断 pkill 对调用方 shell cmdline
# 的自匹配，正则语义不变。
# ============================================================
set -u

PATTERNS=('/opt/ros/humbl[e]' 'simulated_chassis/instal[l]' 'ros2 launc[h]' 'ros2 ru[n]' 'ros2cl[i]' 'ign gaz[ebo]')

list_procs() {
    local pat
    for pat in "${PATTERNS[@]}"; do
        pgrep -af "$pat" || true
    done | sort -u
}

echo "========== [1/5] 清场前 ROS 进程 =========="
list_procs || true

echo ""
echo "========== [2/5] 第一轮 SIGINT（优雅退出，保 SHM 干净）=========="
for pat in "${PATTERNS[@]}"; do pkill -INT -f "$pat" 2>/dev/null || true; done
sleep 4

echo "========== [3/5] 第二轮 SIGTERM / SIGKILL 兜底 =========="
if [ -n "$(list_procs)" ]; then
    for pat in "${PATTERNS[@]}"; do pkill -TERM -f "$pat" 2>/dev/null || true; done
    sleep 3
fi
if [ -n "$(list_procs)" ]; then
    echo "仍有顽固进程，SIGKILL："
    list_procs
    for pat in "${PATTERNS[@]}"; do pkill -KILL -f "$pat" 2>/dev/null || true; done
    sleep 1
fi

echo ""
echo "========== [4/5] 刷新 ros2 daemon（清掉图里的幻影节点）=========="
if ! command -v ros2 >/dev/null 2>&1 && [ -f /opt/ros/humble/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
fi
ros2 daemon stop 2>/dev/null || echo "(ros2 不可用，跳过 daemon 刷新)"

echo ""
echo "========== [5/5] 清理 /dev/shm 的 FastDDS 残留 =========="
if ls /dev/shm/fastrtps_* >/dev/null 2>&1 || ls /dev/shm/sem.fastrtps_* >/dev/null 2>&1; then
    if fuser -s /dev/shm/fastrtps_* 2>/dev/null; then
        echo "⚠️  仍有进程持有 fastrtps 共享内存（上面应有残留进程未杀干净），跳过删除："
        fuser /dev/shm/fastrtps_* 2>/dev/null || true
    else
        rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*
        echo "已删除 /dev/shm/fastrtps_*（端口三件套孤儿是重启后 init_port 报错的根因）"
    fi
else
    echo "/dev/shm 无 fastrtps 残留"
fi

echo ""
echo "========== 完成 =========="
echo "剩余 ROS 进程："
list_procs || echo "  （无）"
