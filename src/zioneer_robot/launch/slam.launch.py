#!/usr/bin/env python3
"""
Cartographer 3D在线建图启动文件
RViz 由 navigation.launch.py 持续托管，定位/建图切换时本 launch 不重复启动 RViz。
ros2 launch zioneer_robot slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config_dir = os.path.join(
        get_package_share_directory('zioneer_robot'),
        'config'
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='slam_3d_online.lua',
            description='Cartographer 3D Lua配置文件'
        ),
    ]

    # ===== Cartographer 3D建图节点 =====
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-start_trajectory_with_default_topics=true',
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

    # ===== 占据栅格地图发布节点（从3D点云投影到2D） =====
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 3D在线建图模式启动（差速+双3D雷达）']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[cartographer_occupancy_grid_node]),

        LogInfo(msg=['3D建图节点已启动，使用 navigation.launch.py 中持续运行的 RViz']),
        LogInfo(msg=['使用键盘控制机器人移动完成建图']),
        LogInfo(msg=['控制按键: i=前进, ,=后退, j=左转, l=右转']),
    ])
