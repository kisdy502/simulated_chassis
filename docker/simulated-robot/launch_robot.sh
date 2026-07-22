#!/bin/bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

ROBOT_TYPE=${ROBOT_TYPE:-"diff_drive"}
MODE=${MODE:-simulation}

echo "=========================================="
echo "Robot: $ROBOT_TYPE"
echo "Mode:  $MODE"
echo "=========================================="

source /ros2_ws/install/setup.bash

case "$MODE" in
    simulation)
        echo "Starting Gazebo simulation..."
        if [ "$ROBOT_TYPE" = "omni_3wd" ]; then
            ros2 launch simulated_chassis simulated_chassis.launch.py
        else
            ros2 launch jzt_robot gazebo_diff_2lidar.launch.py
        fi
        ;;
    slam)
        echo "Starting SLAM mapping..."
        if [ "$ROBOT_TYPE" = "omni_3wd" ]; then
            ros2 launch simulated_chassis slam.launch.py
        else
            ros2 launch jzt_robot slam_online_2lidar.launch.py
        fi
        ;;
    navigation)
        echo "Starting Navigation..."
        PBSTREAM=${PBSTREAM_FILE:-""}
        if [ "$ROBOT_TYPE" = "omni_3wd" ]; then
            if [ -n "$PBSTREAM" ]; then
                ros2 launch simulated_chassis navigation.launch.py pbstream_file:="$PBSTREAM"
            else
                ros2 launch simulated_chassis navigation.launch.py
            fi
        else
            if [ -n "$PBSTREAM" ]; then
                ros2 launch jzt_robot navigation_2lidar.launch.py pbstream_file:="$PBSTREAM"
            else
                ros2 launch jzt_robot navigation_2lidar.launch.py
            fi
        fi
        ;;
    *)
        echo "Unknown mode: $MODE"
        echo "Available modes: simulation, slam, navigation"
        exit 1
        ;;
esac
