FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Shanghai

RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    git \
    ca-certificates \
    lsb-release \
    gnupg \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# ROS 2 Humble apt 源（Ubuntu 22.04 jammy）
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Gazebo Fortress 源（OSRF，ros_ign_gazebo 的 ignition-fortress 依赖来自这里）
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros2launch \
    ros-humble-launch-ros \
    && rm -rf /var/lib/apt/lists/*

# Nav2 + Cartographer + ros2_control（Humble）
# 注意：nav2-mppi-controller 在 Humble 后期版本已回移，MPPI 参数可直接使用
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-nav2-mppi-controller \
    ros-humble-cartographer \
    ros-humble-cartographer-ros \
    ros-humble-ros2-control \
    ros-humble-ign-ros2-control \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-robot-state-publisher \
    ros-humble-xacro \
    ros-humble-behaviortree-cpp-v3 \
    ros-humble-nav2-behavior-tree \
    && rm -rf /var/lib/apt/lists/*

# rosbridge：对外提供 JSON over WebSocket，取代 agv_bridge_v2 自带的 libwebsockets 服务
# 含 rosbridge_server + rosbridge_library + rosapi(+rosapi_msgs)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-rosbridge-suite \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Fortress（ign gazebo / ros_ign_gazebo / ros_gz_bridge）
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-ign-gazebo \
    ros-humble-ros-ign-gazebo-demos \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-ros-gz-interfaces \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    libjsoncpp-dev \
    libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
    vim \
    net-tools \
    iputils-ping \
    htop \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep init || true && rosdep update

ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

COPY docker/ros2-base/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
