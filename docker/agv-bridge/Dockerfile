FROM ros2-base:latest

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

COPY src/agv_bridge_v2/ ./agv_bridge_v2/
COPY src/agv_bridge_v2_interfaces/ ./agv_bridge_v2_interfaces/

WORKDIR /ros2_ws

# 先编接口包（agv_bridge_v2 依赖它生成 action/srv/msg）
RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-up-to agv_bridge_v2 \
    && rm -rf build log

COPY docker/agv-bridge/launch_bridge.sh /launch_bridge.sh
RUN chmod +x /launch_bridge.sh

EXPOSE 9090

ENTRYPOINT ["/launch_bridge.sh"]
