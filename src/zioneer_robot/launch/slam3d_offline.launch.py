#!/usr/bin/env python3
"""
Cartographer 3D 离线建图启动文件
ros2 launch zioneer_robot slam3d_offline.launch.py \
    bag_filenames:=/path/to/my_bag \
    save_state_filename:=/path/to/my_map_optimized.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "zioneer_robot"
    pkg_share = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_share, 'config')
    xacro_path = os.path.join(pkg_share, "urdf", "diff", "robot.xacro")

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
            '-configuration_basenames', LaunchConfiguration('configuration_basename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-save_state_filename', LaunchConfiguration('save_state_filename'),
            '--ros-args',
            '--log-level', 'info',
        ],
        remappings=[
            ('points2_1', '/points2_1'),
            ('points2_2', '/points2_2'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 3D 离线建图模式（差速+双3D雷达）']),
        LogInfo(msg=['从 bag 文件全速处理，生成精细地图']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        cartographer_offline_node,

        LogInfo(msg=['离线建图完成后会自动保存 pbstream']),
    ])
