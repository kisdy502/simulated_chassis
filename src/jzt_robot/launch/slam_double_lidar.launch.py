#!/usr/bin/env python3
"""
Cartographer 建图 + RViz 可视化启动文件

使用方法:
    # 终端1: 先启动gazebo
    ros2 launch jzt_robot gazebo.launch.py

    # 终端2: 再启动建图
    ros2 launch jzt_robot slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 使用 jzt_robot 配置目录
    config_dir = os.path.join(
        get_package_share_directory('jzt_robot'),
        'config'
    )
    
    
    rviz_config = os.path.join(
        get_package_share_directory('jzt_robot'),
        'rviz', 'nav2_double_lidar.rviz'
    )

    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='jzt_robot_2d_double_lidar.lua',
            description='Cartographer Lua配置文件'
        ),
    ]

    # Cartographer 建图节点（双雷达）
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-start_trajectory_with_default_topics', 'true',
        ],
        remappings=[
            ('scan_1', '/scan_front'),
            ('scan_2', '/scan_rear'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )
    
    # 占据栅格地图发布节点
    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-resolution', '0.05',
            '-publish_period_sec', '1.0',
        ],
    )
    
    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
    )
    

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 建图模式启动']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[cartographer_occupancy_grid_node]),
        TimerAction(period=3.0, actions=[rviz_node]),

        LogInfo(msg=['建图节点  + RViz 已启动']),
    ])