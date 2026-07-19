#!/usr/bin/env python3
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable, OpaqueFunction,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def get_world_name(sdf_path: str) -> str:
    """从 SDF 文件中自动提取 world name"""
    tree = ET.parse(sdf_path)
    world_elem = tree.getroot().find("world")
    if world_elem is not None:
        return world_elem.get("name", "default")
    return "default"


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)
    robot_name = 'three_wheel_agv'

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    # 世界文件名（默认 world_m.sdf，可切换为 world_octagon.sdf 等）
    world_arg = DeclareLaunchArgument(
        "world", default_value="world_m.sdf",
        description="Gazebo 世界 SDF 文件名（位于 world/ 目录下）",
    )

    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis.xacro")

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_path]),
            value_type=str,
        )
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

    set_plugin_path = SetEnvironmentVariable(
        "GZ_SIM_SYSTEM_PLUGIN_PATH",
        "/opt/ros/jazzy/lib"
    )
    # 让 Gazebo 能找到本包的本地模型（如 triangular_prism）
    set_resource_path = SetEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        [
            EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
            ":",
            os.path.join(pkg_share, "models"),
        ],
    )

    def world_dependent_actions(context):
        """解析 world 文件后才能确定 gazebo/bridge 的参数，因此放在 OpaqueFunction 里"""
        world_file = context.launch_configurations.get("world", "world_m.sdf")
        world_path = os.path.join(pkg_share, "world", world_file)
        # 兼容传入绝对/相对路径的情况
        if not os.path.isfile(world_path) and os.path.isfile(world_file):
            world_path = world_file
        world_name = get_world_name(world_path)
        clock_gz_topic = f"/world/{world_name}/clock"

        gazebo = ExecuteProcess(
            cmd=["gz", "sim", "-r", "-s", "--render-engine", "ogre",
                 "--render-engine-api-backend", "opengl", world_path],
            output="screen",
        )

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

        bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/lidar/point_cloud/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
                '/lidar/point_cloud@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                f'{clock_gz_topic}@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            ],
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            remappings=[
                (clock_gz_topic, '/clock'),
                ('/lidar/point_cloud/points', 'points2'),
            ],
            output='screen'
        )

        return [gazebo, spawn_after_gazebo, bridge]

    world_opaque = OpaqueFunction(function=world_dependent_actions)

    controller_spawners = TimerAction(
        period=6.0,
        actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
            Node(package="controller_manager", executable="spawner",
                 arguments=["three_wheel_base_controller", "--controller-manager", "/controller_manager"]),
        ],
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twistkeyboard',
        prefix='xterm -e',  # 在独立终端中运行
        remappings=[
            ('/cmd_vel', '/three_wheel_base_controller/cmd_vel'),  # 重映射
        ],
        output='screen',
    )

    # joy 手柄驱动
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    # Gamepad 遥控节点
    gamepad_teleop_node = Node(
        package='simulated_chassis',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/three_wheel_base_controller/cmd_vel',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'watchdog_timeout': 0.8,   # 摇杆断连0.8秒后停车，松手主动发停不会被误杀
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        world_arg,
        set_plugin_path,
        set_resource_path,
        robot_state_pub,
        world_opaque,
        controller_spawners,
        teleop,  # 用游戏手柄替代键盘
        joy_node,
        gamepad_teleop_node,
    ])
