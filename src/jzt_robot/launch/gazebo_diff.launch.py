#!/usr/bin/env python3
"""
Jazzy + Gazebo Garden/Ionic 差速机器人仿真启动文件
参考 simulated_chassis/launch/three_wheel_sim.launch.py

传感器：前270°激光 + 后270°激光 + IMU
驱动方式：ros2_control + diff_drive_controller
"""
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def get_world_name(sdf_path: str) -> str:
    """从 SDF 文件中自动提取 world name"""
    tree = ET.parse(sdf_path)
    world_elem = tree.getroot().find("world")
    if world_elem is not None:
        return world_elem.get("name", "default")
    return "default"


def generate_launch_description():
    pkg_name = "jzt_robot"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )

    # 机器人名称
    robot_name = "diff_agv01"
    world_path = os.path.join(pkg_share, "world", "world_sm.sdf")
    world_name = get_world_name(world_path)  # 从 SDF 自动读取，不用硬编码

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

    # 2. Gazebo
    gazebo = ExecuteProcess(
        cmd=["gz", "sim", "-r", world_path],
        output="screen",
    )

    # 3. 生成机器人 (Gazebo 启动 3 秒后)
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

    # 4. ros2_control 控制器 (Gazebo 启动 6 秒后)
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
    clock_gz_topic = f"/world/{world_name}/clock"
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 前向激光
            f"/model/{robot_name}/laser_front_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            # 后向激光
            f"/model/{robot_name}/laser_rear_link/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
            f"{clock_gz_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            f"/model/{robot_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            f"/model/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            (f"/model/{robot_name}/laser_front_link/scan", "/scan_front"),
            (f"/model/{robot_name}/laser_rear_link/scan", "/scan_rear"),
            (clock_gz_topic, "/clock"),
        ],
        output="screen",
    )

    # 6. 键盘遥控
    teleop = Node(
        package="teleop_twist_keyboard",
        executable="teleop_twist_keyboard",
        name="teleop_twistkeyboard",
        prefix="xterm -e",
        parameters=[{"stamped": True, "frame_id": "base_link"}],
        remappings=[("/cmd_vel", "/diff_drive_controller/cmd_vel")],
        output="screen",
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_plugin_path,
        set_software_render,
        robot_state_pub,
        gazebo,
        spawn_after_gazebo,
        controller_spawners,
        bridge,
        teleop,
    ])