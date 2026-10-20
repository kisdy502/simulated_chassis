#!/usr/bin/env bash
# 编译 lightning 包（lightning-lm 默认带 COLCON_IGNORE，不参与常规 colcon 构建；
# 显式 --packages-select 也发现不了它，须临时摘掉忽略标记再构建）。
#
# 用法：
#   ./scripts/build_lightning.sh          # 默认 -j2（8GB 内存机器 -j8 会 OOM）
#   MAKE_JOBS=8 ./scripts/build_lightning.sh
#
# 首次使用：bash scripts/install_lightning_deps.sh（arm64 / x86_64 均本机编译依赖）。
# 仅检查依赖：bash scripts/build_lightning.sh --check-deps
# 注意：不能用 set -u —— ROS 的 setup.bash 引用未绑定变量会直接退出。
set -eo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IGNORE="$WS/src/lightning-lm/COLCON_IGNORE"
RESTORE_IGNORE=false

if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --check-deps ) ]]; then
    echo "Usage: $0 [--check-deps]" >&2
    exit 2
fi
if [[ ! "${MAKE_JOBS:-2}" =~ ^[1-9][0-9]*$ ]]; then
    echo "MAKE_JOBS must be a positive integer" >&2
    exit 2
fi

cd "$WS"
source /opt/ros/humble/setup.bash
if [ -f install/setup.bash ]; then source install/setup.bash; fi
if [[ -d "$HOME/.local" ]]; then
    export CMAKE_PREFIX_PATH="$HOME/.local${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"
fi
# A removed package or another machine's prefix must not leak into colcon.
prune_prefix_path() {
    local name=$1 value path filtered=
    local -a entries
    value=${!name}
    IFS=: read -r -a entries <<< "$value"
    for path in "${entries[@]}"; do
        if [[ -d "$path" ]]; then
            filtered+="${filtered:+:}$path"
        fi
    done
    export "$name=$filtered"
}
prune_prefix_path AMENT_PREFIX_PATH
prune_prefix_path CMAKE_PREFIX_PATH
export MAKEFLAGS="-j${MAKE_JOBS:-2}"

PROBE=$(mktemp -d)
restore_ignore() {
    if [[ "$RESTORE_IGNORE" == true ]]; then touch "$IGNORE"; fi
    rm -rf -- "$PROBE"
}
trap restore_ignore EXIT

cat > "$PROBE/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(lightning_dependency_probe LANGUAGES CXX)
find_package(Pangolin REQUIRED)
CMAKE
echo "[build_lightning] 本机架构: $(uname -m)，检查 Pangolin 开发依赖..."
if ! cmake -S "$PROBE" -B "$PROBE/build" > "$PROBE/check.log" 2>&1; then
    cat "$PROBE/check.log" >&2
    echo '[build_lightning] Pangolin 缺失或依赖不完整。先运行：' >&2
    echo '  bash scripts/install_lightning_deps.sh' >&2
    echo '  # 自定义安装路径也可通过 Pangolin_DIR / CMAKE_PREFIX_PATH 指定。' >&2
    exit 1
fi
if [[ "${1:-}" == --check-deps ]]; then
    echo '[build_lightning] Pangolin CMake 依赖检查通过（未启动工作区构建）。'
    exit 0
fi

if [ -f "$IGNORE" ]; then
    RESTORE_IGNORE=true
    rm -f "$IGNORE"
    echo "[build_lightning] 已临时移除 COLCON_IGNORE（构建后恢复）"
fi

colcon build --packages-select lightning --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WITH_MARCH_NATIVE=OFF \
    -DCMAKE_EXE_LINKER_FLAGS="-Wl,-rpath,$HOME/.local/lib" \
    -DCMAKE_SHARED_LINKER_FLAGS="-Wl,-rpath,$HOME/.local/lib"

echo "[build_lightning] 完成。可执行文件位于 install/lightning/lib/lightning/"
