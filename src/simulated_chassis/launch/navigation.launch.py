#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer 3D定位 + RViz 可视化启动文件
适用于：三舵轮底盘 + 3D雷达

v2: 定位链路（cartographer + occupancy_grid）拆分到 localization.launch.py，
    本 launch 默认仍包含它（独立使用/docker 部署行为不变）。

使用方法:
    # 使用默认地图（定位 + 导航一体）
    ros2 launch simulated_chassis navigation.launch.py

    # 指定地图文件
    ros2 launch simulated_chassis navigation.launch.py \
        pbstream_file:=/path/to/map.pbstream

    # 真机调试
    ros2 launch simulated_chassis navigation.launch.py \
        use_sim_time:=false

    # 上位机地图管理模式（定位由 agv_nav_server 托管，切图/建图只切定位，nav2 不动）：
    ros2 launch simulated_chassis navigation.launch.py include_localization:=false
    ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
        pbstream_file:=$PWD/maps/my_map.pbstream \
        robot_package:=simulated_chassis
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('simulated_chassis')

    # ===== 文件路径 =====
    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2_params_3d.yaml')
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
        DeclareLaunchArgument(
            'include_localization',
            default_value='true',
            description='是否包含 Cartographer 定位链路。'
                        '上位机切图流程传 false，改由 agv_nav_server 托管定位',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # ===== Cartographer 定位链路（独立 launch，可被 agv_nav_server 单独重启）=====
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'pbstream_file': LaunchConfiguration('pbstream_file'),
            'configuration_basename': LaunchConfiguration('configuration_basename'),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_localization')),
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
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Nav2 导航模式启动（3D定位）']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        # 按顺序启动（给各节点留出启动时间）
        TimerAction(period=0.5, actions=[localization_launch]),
        TimerAction(period=2.5, actions=[cmd_vel_relay]),
        TimerAction(period=3.5, actions=[nav2_launch]),
        TimerAction(period=5.5, actions=[rviz_node]),

        LogInfo(msg=['导航节点已启动']),
        LogInfo(msg=['在 RViz 中设置 2D Goal 启动自主导航']),
        LogInfo(msg=['手柄可随时接管控制']),
    ])
