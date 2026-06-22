#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer 3D定位 + RViz 可视化启动文件
适用于：三舵轮底盘 + 3D雷达

使用方法:
    # 使用默认地图
    ros2 launch simulated_chassis navigation.launch.py

    # 指定地图文件
    ros2 launch simulated_chassis navigation.launch.py \
        pbstream_file:=/path/to/map.pbstream

    # 真机调试
    ros2 launch simulated_chassis navigation.launch.py \
        use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('simulated_chassis')

    # ===== 文件路径 =====
    cartographer_config_dir = os.path.join(pkg_share, 'config')
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2_params_3d.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'cartographer_3d.rviz')
    default_pbstream = os.path.join(pkg_share, 'maps', 'my_map_optimized.pbstream')

    # ===== 启动参数 =====
    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value=default_pbstream,
            description='Cartographer pbstream 地图文件'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='localization_3d.lua',
            description='Cartographer 定位配置文件'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='自动启动 Nav2 导航'
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
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
            '--ros-args',
            '--log-level', 'WARN',
        ],
        remappings=[
            ('points2', '/lidar/point_cloud/points'),
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
        arguments=[
            '-trajectory_id', '0',
            '-min_z', '-0.5',
            '-max_z', '0.5',
            '-z_voxel_size', '0.1',
        ],
    )
    # ===== Nav2 /cmd_vel -> 三舵轮控制器话题转发 =====
    cmd_vel_relay = Node(
        package='simulated_chassis',
        executable='cmd_vel_relay_node',
        name='cmd_vel_relay_node',
        output='screen',
        parameters=[{
            'input_topic': '/cmd_vel',
            'output_topic': '/three_wheel_base_controller/cmd_vel',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    # ===== Nav2 导航（仅 navigation_launch.py） =====
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': nav2_params_file,
            'use_sim_time': use_sim_time,
            'autostart': LaunchConfiguration('autostart'),
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
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Nav2 导航模式启动（3D定位）']),
        LogInfo(msg=['地图: $(var pbstream_file)']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        # 按顺序启动（给各节点留出启动时间）
        TimerAction(period=0.5, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[occupancy_grid_node]),
        TimerAction(period=3.0, actions=[cmd_vel_relay]),
        TimerAction(period=4.0, actions=[nav2_launch]),
        TimerAction(period=6.0, actions=[rviz_node]),

        LogInfo(msg=['导航节点已启动']),
        LogInfo(msg=['在 RViz 中设置 2D Goal 启动自主导航']),
        LogInfo(msg=['手柄可随时接管控制']),
    ])