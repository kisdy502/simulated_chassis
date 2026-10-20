FROM ros2-base:latest

SHELL ["/bin/bash", "-c"]
ARG BUILD_JOBS=2

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV START_RVIZ=false
ENV AGV_PROCESS_LOG_DIR=/ros2_ws/process_logs

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

COPY src/agv_bridge_v2/ ./agv_bridge_v2/
COPY src/agv_map_3d/ ./agv_map_3d/
COPY src/agv_bridge_v2_interfaces/ ./agv_bridge_v2_interfaces/
COPY src/simulated_chassis/ ./simulated_chassis/
COPY src/simulated_ur5e/ ./simulated_ur5e/
COPY src/arm_manager_interfaces/ ./arm_manager_interfaces/
COPY src/arm_manager/ ./arm_manager/
COPY src/simulated_chassis_ur5e/ ./simulated_chassis_ur5e/

WORKDIR /ros2_ws

# 先编译接口包（agv_bridge_v2 依赖它生成 action/srv/msg）
RUN mkdir -p src/simulated_chassis/maps \
    && source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --executor sequential --packages-select agv_bridge_v2_interfaces agv_bridge_v2 agv_map_3d simulated_chassis arm_manager_interfaces arm_manager simulated_ur5e simulated_chassis_ur5e \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

COPY docker/agv-bridge/launch_bridge.sh /launch_bridge.sh
RUN sed -i 's/\r$//' /launch_bridge.sh && chmod +x /launch_bridge.sh
RUN mkdir -p /ros2_ws/maps /ros2_ws/process_logs

EXPOSE 9090 8089

ENTRYPOINT ["/launch_bridge.sh"]
