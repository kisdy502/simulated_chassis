#!/usr/bin/env python3
"""
Cartographer 3D建图启动文件
RViz 由 navigation.launch.py 持续托管，定位/建图切换时本 launch 不重复启动 RViz。
ros2 launch simulated_chassis slam.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
            default_value='slam_3d_online.lua',  # 默认 3D 点云版；2D 雷达版传 slam_2d_lidar_online.lua
            description='Cartographer Lua配置文件（默认 slam_3d_online；2D 传 slam_2d_lidar_online）'
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
            ('scan_1', '/scan_1'),
            ('scan_2', '/scan_2'),
            # 3D 建图：直接消费仿真 bridge 桥出的前后雷达原始 PointCloud2
            # （/front_lidar 与 /rear_lidar 的 gpu_lidar 点云，无过滤节点）
            ('points2_1', '/points2_1'),
            ('points2_2', '/points2_2'),
            ('odom', '/odom'),
            ('imu', '/imu'),
        ],
    )

    # ===== map→odom 重锚定节点 =====
    # cartographer 关闭直发 TF（publish_to_tf=false），改发 /tracked_pose（高频外推位姿）；
    # 本节点在同一时间戳查 odom→base 后发布 map→odom，避免双重计数抖动与低频步进卡顿。
    tracked_pose_tf_node = Node(
        package='simulated_chassis',
        executable='tracked_pose_tf_node',
        name='tracked_pose_tf_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'tracking_frame': 'base_link',
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
            # 1s→5s：建图中 5s 刷新对 RViz 足够，websocket/带宽省 80%
            '-publish_period_sec', '5.0',
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

        TimerAction(period=1.0, actions=[cartographer_node, tracked_pose_tf_node]),
        TimerAction(period=2.0, actions=[cartographer_occupancy_grid_node]),
        # cartographer 一死整组退出：建图会话结束（半成品图本就无法恢复），
        # 避免孤儿 occupancy_grid 继续发 /map 造成地图闪烁。
        RegisterEventHandler(OnProcessExit(
            target_action=cartographer_node,
            on_exit=[EmitEvent(event=Shutdown(reason="cartographer 退出，建图整组关闭"))],
        )),

        LogInfo(msg=['3D建图节点已启动，使用 navigation.launch.py 中持续运行的 RViz']),
        LogInfo(msg=['使用键盘控制机器人移动完成建图']),
        LogInfo(msg=['控制按键: i=前进, ,=后退, j=左转, l=右转']),
    ])
