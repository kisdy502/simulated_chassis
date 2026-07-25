#!/usr/bin/env python3
"""
Cartographer 3D建图 + RViz 可视化启动文件
ros2 launch simulated_chassis slam3d_online.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 使用 simulated_chassis 配置目录
    config_dir = os.path.join(
        get_package_share_directory('simulated_chassis'),
        'config'
    )

    rviz_config = os.path.join(
        get_package_share_directory('simulated_chassis'),
        'rviz', 'cartographer_3d.rviz'  # 新建3D专用RViz配置
    )

    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='slam_3d_online.lua',  # 3D配置文件
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
            '-start_trajectory_with_default_topics', 'true',
            '--ros-args',
            '--log-level', 'info',  # ✅ 添加调试日志
        ],
        remappings=[
            # 3D雷达：点云输入（用 points 或 points_1）
            # ('points', '/lidar/point_cloud/points'),  # 或 '/lidar/point_cloud'
            # ('points_1', '/lidar/point_cloud/points'),
            ('points2', '/points2'),
            ('odom', '/odom'),  # 直接订阅控制器的 odom
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
            '-trajectory_id', '0',        # ✅ 指定轨迹
            # '-min_z', '-0.5',             # ✅ 投影高度范围（地面到50cm）
            # '-max_z', '0.5',
            # '-z_voxel_size', '0.1',
        ],
    )

    # ===== RViz 可视化 =====
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 3D建图模式启动']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[cartographer_occupancy_grid_node]),
        TimerAction(period=3.0, actions=[rviz_node]),

        LogInfo(msg=['3D建图节点 + 键盘控制 + RViz 已启动']),
        LogInfo(msg=['使用键盘控制机器人移动完成建图']),
        LogInfo(msg=['控制按键: i=前进, ,=后退, j=左转, l=右转']),
    ])