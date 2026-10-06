#!/usr/bin/env python3
"""
Cartographer 离线建图启动文件（2D 雷达版机器人）
ros2 launch simulated_chassis slam_offline.launch.py

3D 离线建图：把下方 xacro_path 改为 three_wheel_chassis_3d.xacro，
并传 configuration_basename:=slam_3d_offline.lua（bag 须为 3D 机器人录制）。
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)

    config_dir = os.path.join(pkg_share, 'config')

    # 机器人 xacro 固定为 2D 雷达版；3D 离线建图改为 three_wheel_chassis_3d.xacro
    # （cartographer_offline_node 从 robot_description 自建 TF 树，两版雷达
    #   frame 名不同，必须与录制 bag 的机器人版本一致）
    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis_2d.xacro")

    # 启动参数
    declared_arguments = [
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='slam_2d_lidar_offline.lua',
            description='离线建图配置（默认 slam_2d_lidar_offline；3D 点云版传 slam_3d_offline）'
        ),
        DeclareLaunchArgument(
            'bag_filenames',
            default_value='',
            description='ROS2 bag 文件路径（逗号分隔多个）'
        ),
        DeclareLaunchArgument(
            'save_state_filename',
            default_value='',
            description='保存 pbstream 文件路径'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='If true, use sim time'
        ),
    ]

    # 使用 xacro 动态生成 URDF
    robot_description_content = Command(['xacro ', xacro_path])

    # ===== Cartographer 离线建图节点 =====
    cartographer_offline_node = Node(
        package='cartographer_ros',
        executable='cartographer_offline_node',
        name='cartographer_offline_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_description_content,
        }],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basenames', LaunchConfiguration('configuration_basename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-save_state_filename', LaunchConfiguration('save_state_filename'),
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

    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Cartographer 2D 雷达版离线建图模式']),
        LogInfo(msg=['从 bag 文件全速处理，生成精细地图']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        cartographer_offline_node,

        LogInfo(msg=['离线建图完成后会自动保存 pbstream']),
    ])
