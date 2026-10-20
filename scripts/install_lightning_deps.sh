#!/usr/bin/env bash
# Ubuntu 22.04 / ROS Humble: build the bundled Pangolin for the CURRENT CPU.
# Only apt uses sudo; Pangolin source/build/install stay in the user's directories.
set -euo pipefail
workspace=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
jobs=${MAKE_JOBS:-2}
[[ "$jobs" =~ ^[1-9][0-9]*$ ]] || { echo 'MAKE_JOBS must be a positive integer' >&2; exit 2; }
[[ -f /opt/ros/humble/setup.bash ]] || { echo 'Install ROS 2 Humble first' >&2; exit 1; }
archive="$workspace/src/lightning-lm/thirdparty/Pangolin-0.9.3.zip"
[[ -f "$archive" ]] || { echo "Missing Pangolin source archive: $archive" >&2; exit 1; }
prefix="${PANGOLIN_INSTALL_PREFIX:-$HOME/.local}"
case "$prefix" in /*) ;; *) echo 'PANGOLIN_INSTALL_PREFIX must be absolute' >&2; exit 2 ;; esac
cache="${XDG_CACHE_HOME:-$HOME/.cache}/simulated_chassis/lightning-deps/$(uname -m)"

privilege=()
if [[ $(id -u) -ne 0 ]]; then privilege=(sudo); fi
"${privilege[@]}" apt-get update
"${privilege[@]}" apt-get install -y --no-install-recommends \
    build-essential cmake unzip pkg-config \
    libepoxy-dev libeigen3-dev libgl1-mesa-dev libegl1-mesa-dev libx11-dev libglew-dev \
    libgoogle-glog-dev libgflags-dev libyaml-cpp-dev libtbb-dev libpcl-dev libopencv-dev \
    ros-humble-pcl-conversions ros-humble-rosbag2-cpp

mkdir -p "$cache"
# Extract in a Linux directory, not the Windows mount, to avoid slow metadata I/O.
if [[ ! -f "$cache/Pangolin-0.9.3/CMakeLists.txt" ]]; then
    unzip -q -o "$archive" -d "$cache"
fi
cmake -S "$cache/Pangolin-0.9.3" -B "$cache/build-pangolin-0.9.3" \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_TOOLS=OFF \
    -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DCMAKE_INSTALL_RPATH="$prefix/lib" -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON
cmake --build "$cache/build-pangolin-0.9.3" --parallel "$jobs"
cmake --install "$cache/build-pangolin-0.9.3"
echo "[install_lightning_deps] Pangolin 0.9.3 已按 $(uname -m) 编译安装到 $prefix"
echo '下一步：bash scripts/build_lightning.sh'
if [[ "$prefix" != "$HOME/.local" ]]; then
    echo "自定义前缀需在构建前设置：export CMAKE_PREFIX_PATH=\"$prefix:\${CMAKE_PREFIX_PATH:-}\""
    echo '运行时还需确保动态链接器可以找到此自定义前缀中的共享库。'
fi
