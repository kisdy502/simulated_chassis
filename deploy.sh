#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p maps runtime/bridge/ros runtime/bridge/process runtime/simulation/ros
usage() {
    echo "Usage: $0 {build|up|up-nav|up-slam|start-mapping|save-map NAME|down|restart|logs [SERVICE]|status|shell [SERVICE]}"
}
check_source_files() {
    local path signature source_dir
    local source_dirs=(src/jzt_robot src/simulated_chassis src/agv_bridge_v2 src/agv_bridge_v2_interfaces
        src/arm_manager_interfaces src/arm_manager src/simulated_ur5e src/simulated_chassis_ur5e src/agv_map_3d docker)
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
    *) usage; exit 1 ;;
esac
