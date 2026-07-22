FROM ros2-base:latest

SHELL ["/bin/bash", "-c"]

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    xvfb \
    xterm \
    ros-jazzy-gz-sim-vendor \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-topic-tools \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-rviz2 \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-pointcloud-to-laserscan \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws/src

COPY src/jzt_robot/ ./jzt_robot/
COPY src/simulated_chassis/ ./simulated_chassis/

WORKDIR /ros2_ws

RUN source /opt/ros/jazzy/setup.bash \
    && colcon build --packages-select jzt_robot simulated_chassis \
    && rm -rf build log

ENV GAZEBO_MODEL_PATH=/ros2_ws/src/jzt_robot/models:/ros2_ws/src/simulated_chassis/models
ENV GAZEBO_RESOURCE_PATH=/ros2_ws/src/jzt_robot/world:/ros2_ws/src/simulated_chassis/world
ENV GAZEBO_PLUGIN_PATH=/ros2_ws/install/lib

COPY docker/simulated-robot/launch_robot.sh /launch_robot.sh
COPY docker/simulated-robot/run_rviz.sh /run_rviz.sh
RUN chmod +x /launch_robot.sh /run_rviz.sh

ENTRYPOINT ["/launch_robot.sh"]
