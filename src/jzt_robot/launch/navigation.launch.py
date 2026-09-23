#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer 2D双雷达定位 (带RViz显示)

v2: 定位链路（cartographer + occupancy_grid + pointcloud_to_laserscan）拆分到
    localization.launch.py，本 launch 默认仍包含它（独立使用/docker 部署
    行为不变）。

WSL 单机 + 上位机切图流程（/agv/load_map）：
    ros2 launch jzt_robot navigation.launch.py include_localization:=false
    ros2 launch agv_bridge_v2 agv_rosbridge.launch.py pbstream_file:=$PWD/maps/my_map.pbstream
    （定位由 agv_nav_server 以子进程托管，切图时只重启定位，nav2 不动）
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')

    nav2_params_file = os.path.join(pkg_share, 'param', 'nav2_params_mppi_cartographer_double_lidar.yaml')

    declared_arguments = [
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='',
            description='Cartographer pbstream 地图文件路径（include_localization:=true 时生效）'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用仿真时间'
        ),
        DeclareLaunchArgument(
            'include_localization',
            default_value='true',
            description='是否包含 Cartographer 定位链路。'
                        'WSL 切图流程传 false，改由 agv_nav_server 托管定位',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Cartographer 定位链路（独立 launch，可被 agv_nav_server 单独重启）
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'localization.launch.py')
        ),
        launch_arguments={
            'pbstream_file': LaunchConfiguration('pbstream_file'),
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_localization')),
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

    # 只启动空白 RViz：WSL 中加载完整配置会造成严重卡顿，所需显示项由用户手动添加。
    rviz_node = Node(
        package='rviz2',
        executable='/run_rviz.sh',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    return LaunchDescription([
        LogInfo(msg=['Nav2 + Cartographer Localization + RViz']),
        *declared_arguments,
        TimerAction(period=0.2, actions=[localization_launch]),
        TimerAction(period=1.0, actions=[nav2_launch]),
        TimerAction(period=2.0, actions=[rviz_node]),
    ])
