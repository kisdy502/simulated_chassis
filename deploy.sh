#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p maps runtime/bridge/ros runtime/bridge/process runtime/simulation/ros
usage() {
    echo "Usage: $0 {build|up|up-nav|up-slam|start-mapping|save-map NAME|down|restart|logs [SERVICE]|status|export [FILE]|bundle [DIRECTORY]|shell [SERVICE]}"
}
check_source_files() {
    local path signature source_dir
    local source_dirs=(src/jzt_robot src/simulated_chassis src/agv_bridge_v2 src/agv_bridge_v2_interfaces
        src/arm_manager_interfaces src/arm_manager src/simulated_ur5e src/simulated_chassis_ur5e docker)
    for source_dir in "${source_dirs[@]}"; do
        [[ -d "$source_dir" ]] || { echo "Missing build source directory: $source_dir" >&2; return 1; }
    done
    while IFS= read -r -d '' path; do
        signature=$(od -An -tx1 -N3 "$path" | tr -d ' \n')
        if [[ "$signature" == 887d1c ]]; then
            echo "Encrypted file is unreadable to Docker: $path. Restore Linux-readable plain text before building." >&2
            return 1
        fi
    done < <(find "${source_dirs[@]}" -type f -print0)
}
call_bridge() {
    local service=$1 type=$2 request=$3 output
    output=$(docker compose exec -T agv-bridge bash -c '
        source /opt/ros/humble/setup.bash
        source /ros2_ws/install/setup.bash
        exec timeout 60 ros2 service call "$1" "$2" "$3"
    ' bash "$service" "$type" "$request")
    printf '%s\n' "$output"
    if ! printf '%s\n' "$output" | grep -Eq 'success=True|success: true'; then
        echo "Bridge rejected the request or no successful response was received" >&2
        return 1
    fi
}
case "${1:-}" in
    build)
        check_source_files
        docker build -t ros2-base:latest -f docker/ros2-base/Dockerfile .
        docker compose build robot-simulation agv-bridge
        ;;
    up|up-nav) docker compose up -d robot-simulation agv-bridge ;;
    up-slam)
        docker compose up -d robot-simulation agv-bridge
        call_bridge /agv/start_mapping agv_bridge_v2_interfaces/srv/StartMapping '{}'
        ;;
    start-mapping)
        call_bridge /agv/start_mapping agv_bridge_v2_interfaces/srv/StartMapping '{}'
        ;;
    save-map)
        name=${2:?Specify a map name}
        [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { echo "Use letters, digits, underscores or hyphens" >&2; exit 1; }
        call_bridge /agv/save_map agv_bridge_v2_interfaces/srv/SaveMap "{map_name: '$name'}"
        echo "Save accepted asynchronously. Check /agv/status and bridge logs; files are under maps/$name/."
        ;;
    down) docker compose down --remove-orphans ;;
    restart) docker compose up -d --force-recreate robot-simulation agv-bridge ;;
    logs) docker compose logs -f "${2:-agv-bridge}" ;;
    status) docker compose ps ;;
    shell) docker compose exec "${2:-agv-bridge}" bash ;;
    bundle)
        directory=${2:-delivery/wheel-3d-ur5e}
        [[ ! -e "$directory" ]] || { echo "Delivery directory already exists: $directory" >&2; exit 1; }
        # Validate every image before creating the delivery folder.
        docker image inspect ros2-base:latest simulated-robot:wheel-3d agv-bridge:wheel-3d >/dev/null
        for image in simulated-robot:wheel-3d agv-bridge:wheel-3d; do
            if ! docker run --rm --network none --entrypoint bash "$image" -c '
                source /opt/ros/humble/setup.bash
                source /ros2_ws/install/setup.bash
                ros2 pkg prefix simulated_chassis_ur5e >/dev/null && ros2 pkg prefix arm_manager >/dev/null
            '; then
                echo "Image lacks assembled arm runtime: $image. Run deploy.sh build first." >&2
                exit 1
            fi
        done
        mkdir -p "$directory/docker" "$directory/maps"
        cp docker-compose.yml deploy.sh .env.example "$directory/"
        cp .env.example "$directory/.env"
        cp docker/cyclonedds.xml docker/README.md "$directory/docker/"
        if [[ "${INCLUDE_MAPS:-true}" == true ]]; then
            # A new simulation must not inherit the sender's last parked pose.
            find maps -type f ! -name last_pose.yaml -printf '%P\0' |
                tar -C maps --null --no-recursion -T - -cf - | tar -C "$directory/maps" -xf -
        fi
        docker image save ros2-base:latest simulated-robot:wheel-3d agv-bridge:wheel-3d |
            gzip > "$directory/wheel-3d.docker-images.tar.gz"
        docker image inspect --format '{{.RepoTags}} {{.Id}}' \
            ros2-base:latest simulated-robot:wheel-3d agv-bridge:wheel-3d > "$directory/images.txt"
        (cd "$directory" && sha256sum wheel-3d.docker-images.tar.gz > SHA256SUMS)
        cat > "$directory/START_HERE.txt" <<'DELIVERY'
三舵轮 + UR5e 离线仿真交付包

请在接收方自己的 Linux Docker 主机运行，发送方关闭 WSL 不影响独立主机。
依赖：Docker Engine、Docker Compose v2、bash、gzip；镜像平台为 linux/amd64。
在交付目录执行：
1. sha256sum -c SHA256SUMS
2. gzip -dc wheel-3d.docker-images.tar.gz | docker load
3. 修改 .env，保留 ROBOT_PROFILE=chassis_3d_ur5e。
   若使用随包地图，设置：
   PBSTREAM_FILE=/ros2_ws/maps/地图名/地图名.pbstream
   留空则不自动定位，可通过上位机启动建图。
4. bash deploy.sh up
5. 上位机连接 ws://接收方主机IP:9090（或 .env 中配置的端口）。

查看日志：bash deploy.sh logs
停止服务：bash deploy.sh down
地图保存在 maps/，不携带发送方的 last_pose.yaml 停车位姿记忆。
首次定位请确认点云贴合，必要时通过上位机设置初始位置。
示教数据和日志保存在运行后生成的 runtime/。
这两个容器不包含上位机/调度服务器，上位机请独立部署或连接已有服务。
无需安装 ROS，无需源码编译；交付包不含源码，不要执行 deploy.sh build。
不要在同一 ROS 域另外启动机械臂、Gazebo 或底盘控制器。
更多说明见 docker/README.md。

DELIVERY
        echo "Offline delivery ready: $directory (contains maps unless INCLUDE_MAPS=false)"
        ;;
    export)
        file=${2:-wheel-3d.docker-images.tar.gz}
        docker image save ros2-base:latest simulated-robot:wheel-3d agv-bridge:wheel-3d | gzip > "$file"
        echo "Exported $file; also deliver docker-compose.yml, docker/, deploy.sh and .env.example. Maps are separate."
        ;;
    *) usage; exit 1 ;;
esac
