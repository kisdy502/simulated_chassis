#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fi
if [[ "${MODE:-simulation}" != simulation ]]; then
    echo "Mapping and navigation are managed by agv-bridge; MODE must be simulation" >&2
    exit 1
fi
if [[ "${ROBOT_TYPE:-omni_3wd}" != omni_3wd ]]; then
    echo "This wheel-3d deployment requires ROBOT_TYPE=omni_3wd" >&2
    exit 1
fi
args=(ros2 launch simulated_chassis three_wheel_sim.launch.py
      use_sim_time:="${USE_SIM_TIME:-true}" include_cmd_vel_relay:=false
      start_teleop:=false start_gamepad:=false gui:="${GAZEBO_GUI:-false}")
if [[ "${GAZEBO_GUI:-false}" == true ]]; then
    exec "${args[@]}"
else
    # GPU lidar still needs a rendering context in server-only mode.
    export LIBGL_ALWAYS_SOFTWARE=1
    exec xvfb-run -a -s "-screen 0 1280x720x24" "${args[@]}"
fi
