#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer 2D双雷达定位 (带RViz显示)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')

    cartographer_config_dir = os.path.join(pkg_share, 'config')
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2_params_mppi_cartographer_double_lidar.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_double_lidar.rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='',
            description='Cartographer pbstream 地图文件路径'
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

    # Nav2 导航
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )

    # RViz 可视化 (使用包装脚本强制软件渲染)
    rviz_node = Node(
        package='rviz2',
        executable='/run_rviz.sh',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['Nav2 + Cartographer Localization + RViz']),
        *declared_arguments,
        TimerAction(period=0.2, actions=[cartographer_node]),
        TimerAction(period=1.0, actions=[occupancy_grid_node]),
        TimerAction(period=2.0, actions=[nav2_launch]),
        TimerAction(period=3.0, actions=[rviz_node, pointcloud_to_laserscan_node]),
    ])
