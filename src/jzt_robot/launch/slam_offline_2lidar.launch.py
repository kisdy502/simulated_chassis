#!/usr/bin/env python3
"""
Cartographer 2D 双雷达离线建图启动文件
从 ROS2 bag 文件全速处理，生成高质量 pbstream 地图

用法:
    ros2 launch jzt_robot slam_offline_2lidar.launch.py \
        bag_filenames:=/path/to/bag_file \
        save_state_filename:=/path/to/map.pbstream
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "jzt_robot"
    pkg_share = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_share, 'config')

    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='slam_2d_offline.lua',
            description='Cartographer 2D 离线配置文件'
        ),
        DeclareLaunchArgument(
            'bag_filenames',
            default_value='',
            description='ROS2 bag 文件路径（逗号分隔多个 bag）'
        ),
        DeclareLaunchArgument(
            'save_state_filename',
            default_value='',
            description='保存 pbstream 地图文件路径'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用 bag 中的仿真时间'
        ),
    ]

    # Cartographer 离线建图节点
    cartographer_offline_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        name='cartographer_offline_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 2D 双雷达 离线建图模式']),
        LogInfo(msg=['从 bag 文件全速处理，生成精细地图']),
        LogInfo(msg=['==========================================']),
        *declared_arguments,
        cartographer_offline_node,
        LogInfo(msg=['离线建图完成后自动保存 pbstream']),
    ])
