#!/usr/bin/env python3
"""
Cartographer 2D 双雷达纯定位（可独立重启的最小单元）

只包含定位链路：cartographer_node + occupancy_grid_node + pointcloud_to_laserscan。
与 nav2 解耦 —— 上位机 /agv/load_map 切换地图时，agv_nav_server 只杀掉并重拉
这一组节点，nav2 全栈（控制器/规划器/代价地图）不重启。

用法：
  ros2 launch jzt_robot localization.launch.py pbstream_file:=/abs/path/my_map.pbstream
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')
    cartographer_config_dir = os.path.join(pkg_share, 'config')

    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='',
            description='Cartographer pbstream 地图文件路径（纯定位必须提供）'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Cartographer 定位节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', 'localization_2d.lua',
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
        ],
        remappings=[
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    # 占据栅格地图发布
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
        }],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        parameters=[{
            "use_sim_time": use_sim_time,
            "target_frame": "base_link",
            "transform_tolerance": 0.01,
            "min_height": 0.0,
            "max_height": 0.1,
            "angle_min": -3.14159,
            "angle_max": 3.14159,
            "angle_increment": 0.01745,
            "scan_time": 0.333,
            "range_min": 0.1,
            "range_max": 10.0,
            "use_inf": True,
        }],
        remappings=[
            ("/cloud_in", "/scan_matched_points2"),
            ("/scan", "/scan"),
        ],
        output="screen",
    )

    return LaunchDescription([
        LogInfo(msg=['Cartographer Localization（独立重启单元，与 nav2 解耦）']),
        *declared_arguments,
        TimerAction(period=0.0, actions=[cartographer_node]),
        TimerAction(period=0.5, actions=[occupancy_grid_node]),
        TimerAction(period=1.0, actions=[pointcloud_to_laserscan_node]),
    ])
