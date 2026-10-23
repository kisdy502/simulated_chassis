#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fi
export AGV_PROCESS_LOG_DIR=${AGV_PROCESS_LOG_DIR:-/ros2_ws/process_logs}
mkdir -p "$AGV_PROCESS_LOG_DIR" "${MAPS_DIR:-/ros2_ws/maps}"
pbstream=${PBSTREAM_FILE:-}
algorithm=${SLAM_ALGORITHM:-cartographer}
case "$algorithm" in
    lightning|cartographer) ;;
    *) echo "Unsupported SLAM_ALGORITHM: $algorithm" >&2; exit 1 ;;
esac
if [[ -n "$pbstream" && ! -e "$pbstream" ]]; then
    echo "Map does not exist: $pbstream" >&2
    exit 1
fi
args=(
    agv_id:="${AGV_ID:-AGV001}" port:="${WEBSOCKET_PORT:-9090}"
    use_sim_time:="${USE_SIM_TIME:-true}"
    slam_algorithm:="$algorithm"
    robot_package:="${ROBOT_PACKAGE:-simulated_chassis}"
    maps_dir:="${MAPS_DIR:-/ros2_ws/maps}"
    manage_navigation:="${MANAGE_NAVIGATION:-true}"
    back_up_max_heading_error_deg:="${BACK_UP_MAX_HEADING_ERROR_DEG:-20.0}"
)
# ros2 launch rejects an empty CLI value; use the launch default for no initial map.
if [[ -n "$pbstream" ]]; then args+=(pbstream_file:="$pbstream"); fi
if [[ "${ROBOT_PROFILE:-chassis_3d_ur5e}" == chassis_3d_ur5e ]]; then
    # This profile fixes robot_package and use_sim_time internally.
    profile_args=(agv_id:="${AGV_ID:-AGV001}" port:="${WEBSOCKET_PORT:-9090}"
      maps_dir:="${MAPS_DIR:-/ros2_ws/maps}" manage_navigation:="${MANAGE_NAVIGATION:-true}"
      back_up_max_heading_error_deg:="${BACK_UP_MAX_HEADING_ERROR_DEG:-20.0}"
      rviz:="${START_RVIZ:-false}")
    profile_args+=(slam_algorithm:="$algorithm")
    if [[ -n "$pbstream" ]]; then profile_args+=(pbstream_file:="$pbstream"); fi
    exec ros2 launch simulated_chassis_ur5e bridge.launch.py "${profile_args[@]}"
elif [[ "${ROBOT_PROFILE}" == chassis_3d ]]; then
    exec ros2 launch agv_bridge_v2 agv_rosbridge.launch.py "${args[@]}"
else
    echo "Unsupported ROBOT_PROFILE: ${ROBOT_PROFILE}" >&2
    exit 1
fi
