#!/usr/bin/env bash
# 编译 lightning 包（lightning-lm 默认带 COLCON_IGNORE，不参与常规 colcon 构建；
# 显式 --packages-select 也发现不了它，须临时摘掉忽略标记再构建）。
#
# 用法：
#   ./scripts/build_lightning.sh          # 默认 -j2（8GB 内存机器 -j8 会 OOM）
#   MAKE_JOBS=8 ./scripts/build_lightning.sh
#
# 前提：Pangolin/libepoxy 已安装到 ~/.local（见 docs/lightning_vs_cartographer.md §2）。
# 注意：不能用 set -u —— ROS 的 setup.bash 引用未绑定变量会直接退出。
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IGNORE="$WS/src/lightning-lm/COLCON_IGNORE"
MARKER="/tmp/.lightning_build_marker.$$"

cd "$WS"
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
export MAKEFLAGS="-j${MAKE_JOBS:-2}"

restore_ignore() {
    if [ -f "$MARKER" ]; then touch "$IGNORE"; rm -f "$MARKER"; fi
}
trap restore_ignore EXIT

if [ -f "$IGNORE" ]; then
    rm -f "$IGNORE"
    touch "$MARKER"
    echo "[build_lightning] 已临时移除 COLCON_IGNORE（构建后恢复）"
fi

colcon build --packages-select lightning --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXE_LINKER_FLAGS="-Wl,-rpath,$HOME/.local/lib" \
    -DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath,$HOME/.local/lib"

echo "[build_lightning] 完成。可执行文件位于 install/lightning/lib/lightning/"
