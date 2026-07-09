#!/usr/bin/env python3
"""
Cartographer 在线SLAM建图启动文件
适配 Gazebo 仿真机器人 jztDiffRobot

使用方法:
    ros2 launch cartographer_slam.launch.py

建图过程中可通过rviz查看地图构建进度。

Author: CodeBuddy
Date: 2026-04-24
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Cartographer配置目录
    pkg_share = get_package_share_directory('cartographer_ros')
    config_dir = os.path.join(pkg_share, 'configuration_files')
    
    # 默认使用在线SLAM配置文件
    configuration_basename = LaunchConfiguration(
        'configuration_basename', 
        default='jzt_robot_2d.lua'
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
            default_value='backpack_2d.lua',
            description='Cartographer Lua配置文件'
        ),
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/scan',
            description='激光雷达话题'
        ),
    ]
    
    # ============================================================
    # Cartographer SLAM 节点
    # ============================================================
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', configuration_basename,
            '-start_trajectory_with_default_topics', 'false',
        ],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom'),
        ],
    )
    
    # ============================================================
    # 占据栅格地图发布节点 (实时显示建图结果)
    # ============================================================
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'resolution': 0.05,  # 5cm分辨率
            'publish_period_sec': 1.0,
        }],
    )
    
    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 在线SLAM 启动']),
        LogInfo(msg=['激光雷达话题: /scan']),
        LogInfo(msg=['里程计话题: /odom']),
        LogInfo(msg=['==========================================']),
        
        *declared_arguments,
        
        # 等待1秒后启动Cartographer
        TimerAction(
            period=1.0,
            actions=[cartographer_node],
        ),
        
        # 等待2秒后启动地图发布
        TimerAction(
            period=2.0,
            actions=[occupancy_grid_node],
        ),
        
        LogInfo(msg=['建图节点已启动，请控制机器人在环境中移动...']),
    ])
