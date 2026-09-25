#!/usr/bin/env python3
"""
Cartographer 3D建图启动文件
RViz 由 navigation.launch.py 持续托管，定位/建图切换时本 launch 不重复启动 RViz。
ros2 launch simulated_chassis slam.launch.py
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
            '-start_trajectory_with_default_topics=true',
            '--ros-args',
            '--log-level', 'info',  # ✅ 添加调试日志
        ],
        remappings=[
            # 双3D雷达：前→points2_1，后→points2_2（由 ros_gz_bridge 桥接并提供）
            # ⚠️ 订阅滤波后话题：地面回波已被 pointcloud_ground_filter 滤除，
            #    否则地面点会投到2D地图上成灰色噪点，并干扰3D匹配的z方向
            ('points2_1', '/points2_1_filtered'),
            ('points2_2', '/points2_2_filtered'),
            ('odom', '/odom'),  # 直接订阅控制器的 odom
            ('imu', '/imu'),
        ],
    )

    # ===== 点云地面滤除节点 =====
    # 雷达距地仅0.25m，垂直下沿-10°(前后倾5°后约-15°)，1~3m内即打到地面。
    # 在 base_footprint 系裁剪高度带，只留 [min_z, max_z] 的点给 Cartographer。
    ground_filter_node = Node(
        package='simulated_chassis',
        executable='pointcloud_ground_filter',
        name='pointcloud_ground_filter',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'target_frame': 'base_footprint',
            'min_z': 0.10,   # 地面上方10cm以下丢弃（地面回波+自身底盘近场）
            'max_z': 3.0,    # 天花板以上不参与建图
        }],
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
            # ⚠️ 源码核对：cartographer_ros/cartographer_ros/occupancy_grid_node_main.cc
            #    中 DEFINE_ 的全部 flag 只有 5 个：resolution / publish_period_sec /
            #    include_frozen_submaps / include_unfrozen_submaps / occupancy_grid_topic。
            #    下面 4 个参数源码里根本不存在，节点启动会打 "Unknown flag" 警告并静默忽略：
            # '-trajectory_id', '0',
            # '-min_z', '-0.05',
            # '-max_z', '1.5',
            # '-z_voxel_size', '0.1',
        ],
    )

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 3D建图模式启动']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=0.3, actions=[ground_filter_node]),
        TimerAction(period=1.0, actions=[cartographer_node]),
        TimerAction(period=2.0, actions=[cartographer_occupancy_grid_node]),

        LogInfo(msg=['3D建图节点已启动，使用 navigation.launch.py 中持续运行的 RViz']),
        LogInfo(msg=['使用键盘控制机器人移动完成建图']),
        LogInfo(msg=['控制按键: i=前进, ,=后退, j=左转, l=右转']),
    ])
