#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source /ros2_ws/install/setup.bash

export AGV_ID=${AGV_ID:-"AGV001"}
export SPRINGBOOT_HOST=${SPRINGBOOT_HOST:-"upper-computer"}
export SPRINGBOOT_PORT=${SPRINGBOOT_PORT:-22777}
export WEBSOCKET_PORT=${WEBSOCKET_PORT:-9090}

echo "Starting AGV Bridge Node"
echo "  AGV_ID: $AGV_ID"
echo "  SpringBoot: $SPRINGBOOT_HOST:$SPRINGBOOT_PORT"
echo "  WebSocket Port: $WEBSOCKET_PORT"

ros2 run agv_bridge_v2 agv_bridge_node --ros-args \
    -p agv_id:=$AGV_ID \
    -p springboot_host:=$SPRINGBOOT_HOST \
    -p springboot_port:=$SPRINGBOOT_PORT \
    -p websocket_port:=$WEBSOCKET_PORT
