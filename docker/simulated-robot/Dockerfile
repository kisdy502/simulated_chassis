FROM ros2-base:latest

SHELL ["/bin/bash", "-c"]
ARG BUILD_JOBS=2

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    xvfb \
    xauth \
    xterm \
    ros-humble-teleop-twist-keyboard \
    ros-humble-topic-tools \
    ros-humble-rviz2 \
    ros-humble-joint-state-publisher-gui \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-cv-bridge \
    ros-humble-std-srvs \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

COPY src/jzt_robot/ ./jzt_robot/
COPY src/simulated_chassis/ ./simulated_chassis/
COPY src/simulated_ur5e/ ./simulated_ur5e/
COPY src/arm_manager_interfaces/ ./arm_manager_interfaces/
COPY src/agv_bridge_v2_interfaces/ ./agv_bridge_v2_interfaces/
COPY src/arm_manager/ ./arm_manager/
COPY src/simulated_chassis_ur5e/ ./simulated_chassis_ur5e/

WORKDIR /ros2_ws

RUN mkdir -p src/jzt_robot/maps src/simulated_chassis/maps \
    && source /opt/ros/humble/setup.bash \
    && MAKEFLAGS="-j${BUILD_JOBS}" colcon build --executor sequential --packages-select jzt_robot simulated_chassis agv_bridge_v2_interfaces arm_manager_interfaces arm_manager simulated_ur5e simulated_chassis_ur5e \
        --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    && rm -rf build log

ENV GAZEBO_MODEL_PATH=/ros2_ws/src/jzt_robot/models:/ros2_ws/src/simulated_chassis/models
ENV GAZEBO_RESOURCE_PATH=/ros2_ws/src/jzt_robot/world:/ros2_ws/src/simulated_chassis/world
ENV GAZEBO_PLUGIN_PATH=/ros2_ws/install/lib

COPY docker/simulated-robot/launch_robot.sh /launch_robot.sh
COPY docker/simulated-robot/run_rviz.sh /run_rviz.sh
RUN sed -i 's/\r$//' /launch_robot.sh /run_rviz.sh && chmod +x /launch_robot.sh /run_rviz.sh

ENTRYPOINT ["/launch_robot.sh"]
