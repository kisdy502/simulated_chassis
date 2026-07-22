FROM ros2-base:latest

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

COPY src/agv_bridge_v2/ ./agv_bridge_v2/

WORKDIR /ros2_ws

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select agv_bridge_v2 \
    && rm -rf build log

COPY docker/agv-bridge/launch_bridge.sh /launch_bridge.sh
RUN chmod +x /launch_bridge.sh

EXPOSE 9090

ENTRYPOINT ["/launch_bridge.sh"]
