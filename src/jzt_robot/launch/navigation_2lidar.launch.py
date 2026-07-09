#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer 2D双雷达定位 + RViz 可视化

用法:
    ros2 launch jzt_robot navigation_2lidar.launch.py \
        pbstream_file:=/path/to/map.pbstream use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')

    # ===== 文件路径 =====
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2_params_mppi_cartographer_double_lidar.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_double_lidar.rviz')

    # ===== 启动参数 =====
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

    # ===== Cartographer 定位节点 =====
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
            '--ros-args',
            '--log-level', 'WARN',
        ],
        remappings=[
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    # ===== 占据栅格地图发布 =====
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
            "target_frame": "base_link",      # 输出的 LaserScan 的 frame_id
            "transform_tolerance": 0.01,
            "min_height": 0.0,                # 只取 z=0 附近的点
            "max_height": 0.1,
            "angle_min": -3.14159,            # -180°
            "angle_max": 3.14159,             # +180°
            "angle_increment": 0.01745,       # 1°
            "scan_time": 0.333,
            "range_min": 0.1,
            "range_max": 10.0,
            "use_inf": True,
        }],
        remappings=[
            ("/cloud_in", "/scan_matched_points2"),   # 输入：Cartographer 的点云
            ("/scan", "/scan"),           # 输出：转换后的 LaserScan
        ],
        output="screen",
    )

    # ===== Nav2 导航 =====
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

    # ===== RViz 可视化 =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['Nav2 导航 + Cartographer 双雷达定位']),
        *declared_arguments,
        TimerAction(period=0.2, actions=[cartographer_node]),
        TimerAction(period=1.0, actions=[occupancy_grid_node]),
        TimerAction(period=2.0, actions=[nav2_launch]),
        TimerAction(period=3.0, actions=[rviz_node,pointcloud_to_laserscan_node]),
    ])
