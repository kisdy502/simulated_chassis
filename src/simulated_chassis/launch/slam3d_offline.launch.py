#!/usr/bin/env python3
"""
Cartographer 3D 离线建图启动文件
ros2 launch simulated_chassis slam3d_offline.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_share, 'config')
    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis.xacro")

    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='slam_3d_offline.lua',
            description='Cartographer 3D 离线配置文件'
        ),
        DeclareLaunchArgument(
            'bag_filenames',
            default_value='',
            description='ROS2 bag 文件路径（逗号分隔多个）'
        ),
        DeclareLaunchArgument(
            'save_state_filename',
            default_value='',
            description='保存 pbstream 文件路径'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use sim time'
        ),
    ]

    # 使用 xacro 动态生成 URDF
    robot_description_content = Command(['xacro ', xacro_path])

    # ===== Cartographer 离线建图节点 =====
    cartographer_offline_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        name='cartographer_offline_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basenames', LaunchConfiguration('configuration_basename'),  # ✅ 改这里
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-save_state_filename', LaunchConfiguration('save_state_filename'),
            '--ros-args',
            '--log-level', 'info',
        ],
        remappings=[
            ('points2', '/points2'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 3D 离线建图模式']),
        LogInfo(msg=['从 bag 文件全速处理，生成精细地图']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        cartographer_offline_node,

        LogInfo(msg=['离线建图完成后会自动保存 pbstream']),
    ])