#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
if [[ "$RMW_IMPLEMENTATION" == rmw_cyclonedds_cpp ]]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE FASTDDS_DEFAULT_PROFILES_FILE
fi
exec "$@"
