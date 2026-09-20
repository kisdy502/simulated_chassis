#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /ros2_ws/install/setup.bash

export AGV_ID=${AGV_ID:-"AGV001"}
export WEBSOCKET_PORT=${WEBSOCKET_PORT:-9090}
export USE_SIM_TIME=${USE_SIM_TIME:-true}
export BACK_UP_MAX_HEADING_ERROR_DEG=${BACK_UP_MAX_HEADING_ERROR_DEG:-20.0}

echo "Starting AGV Nav Server + rosbridge"
echo "  AGV_ID: $AGV_ID"
echo "  rosbridge Port: $WEBSOCKET_PORT"
echo "  上位机接入: ws://<host>:$WEBSOCKET_PORT"

exec ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
    agv_id:=$AGV_ID \
    port:=$WEBSOCKET_PORT \
    use_sim_time:=$USE_SIM_TIME \
    back_up_max_heading_error_deg:=$BACK_UP_MAX_HEADING_ERROR_DEG
