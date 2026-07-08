#!/usr/bin/env python3
"""
Jazzy + Gazebo Garden/Ionic 差速机器人仿真启动文件

传感器：前270°激光 + 后270°激光 + IMU
驱动方式：ros2_control + diff_drive_controller
"""
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
    pkg_name = "jzt_robot"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )

    # 机器人名称
    robot_name = "jzDiffRobot"
    world_name_arg = DeclareLaunchArgument(
        "world", default_value="empty.sdf", description="Gazebo world 文件"
    )

    xacro_path = os.path.join(pkg_share, "urdf", "diff", "robot.xacro")
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path])
    }

    # 环境变量
    set_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH", "/opt/ros/jazzy/lib"
    )
    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    # 1. robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # 2. Gazebo (使用 empty world 或自定义 world)
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", LaunchConfiguration("world")],
        output="screen",
    )

    # 3. 生成机器人 (Gazebo 启动3秒后)
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )

    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=gazebo,
            on_start=[TimerAction(period=3.0, actions=[spawn_robot])],
        )
    )

    # 4. ros2_control 控制器 (生成后3秒)
    controller_spawners = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            ),
        ],
    )

    # 5. ros_gz_bridge: Gazebo 话题 → ROS2 话题
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 前向激光
            f"/model/{robot_name}/laser_front_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # 后向激光
            f"/model/{robot_name}/laser_rear_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # IMU
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            # 时钟
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            # 里程计
            f"/model/{robot_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            (f"/model/{robot_name}/laser_front_link/scan", "/scan_front"),
            (f"/model/{robot_name}/laser_rear_link/scan", "/scan_rear"),
        ],
        output="screen",
    )

    # 6. 键盘遥控
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twistkeyboard",
        prefix="xterm -e",
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        output="screen",
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_name_arg)
    ld.add_action(set_plugin_path)
    ld.add_action(set_software_render)
    ld.add_action(robot_state_pub)
    ld.add_action(gazebo)
    ld.add_action(spawn_after_gazebo)
    ld.add_action(controller_spawners)
    ld.add_action(bridge)
    ld.add_action(teleop)

    return ld