#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )

    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis.xacro")
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path])
    }

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")
    set_gazebo_plugin_path = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib"
    )


    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "empty.sdf"],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", "three_wheel_agv",
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.5",
        ],
        output="screen",
    )

    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=ign_gazebo,
            on_start=[TimerAction(period=3.0, actions=[spawn_robot])],
        )
    )

    controller_spawners = TimerAction(
        period=8.0,
        actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
            Node(package="controller_manager", executable="spawner",
                 arguments=["three_wheel_base_controller", "--controller-manager", "/controller_manager"]),
        ],
    )
    
    # ✅ 修复：使用 Edifice 的桥接包
    bridge = Node(
        package='ros_ign_bridge',  # 改：ros_gz_bridge → ros_ign_bridge
        executable='parameter_bridge',
        arguments=[
            '/lidar_points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',  # 改：gz.msgs → ignition.msgs
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',  # 改：gz.msgs → ignition.msgs
            '/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',  # 改：gz.msgs → ignition.msgs
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_software_render,
        set_gazebo_plugin_path,
        robot_state_pub,
        ign_gazebo,
        spawn_after_gazebo,
        controller_spawners,
        bridge
    ])
