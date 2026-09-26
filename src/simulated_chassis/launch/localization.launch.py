#!/usr/bin/env python3
"""
Cartographer 3D 纯定位（可独立重启的最小单元）
适用于：三舵轮底盘 + 3D雷达

只包含定位链路：cartographer_node + occupancy_grid_node。
与 nav2 解耦 —— 上位机 /agv/load_map 切换地图时，agv_nav_server 只杀掉并重拉
这一组节点，nav2 全栈（控制器/规划器/代价地图）不重启。

用法：
  ros2 launch simulated_chassis localization.launch.py \
      pbstream_file:=/abs/path/my_map.pbstream
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("simulated_chassis")

    cartographer_config_dir = os.path.join(pkg_share, "config")
    default_pbstream = os.path.join(pkg_share, "maps", "my_map_optimized.pbstream")

    declared_arguments = [
        DeclareLaunchArgument(
            "pbstream_file",
            default_value=default_pbstream,
            description="Cartographer pbstream 地图文件",
        ),
        DeclareLaunchArgument(
            "configuration_basename",
            default_value="localization_2d.lua",  # 默认 2D（水平扫描版）；3D 传 localization_3d.lua
            description="Cartographer 定位配置文件",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="使用仿真时间"
        ),
        DeclareLaunchArgument(
            "start_trajectory_with_default_topics",
            default_value="true",
            description="是否自动从零位姿启动轨迹；bridge 托管时设为 false",
        ),
    ]

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ===== Cartographer 定位节点 =====
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            cartographer_config_dir,
            "-configuration_basename",
            LaunchConfiguration("configuration_basename"),
            "-load_state_filename",
            LaunchConfiguration("pbstream_file"),
            # gflags 的 bool flag 不支持空格分隔（"-flag false" 会解析成 true），
            # 必须等号连写成单个 argv token："-flag=false"
            [
                "-start_trajectory_with_default_topics=",
                LaunchConfiguration("start_trajectory_with_default_topics"),
            ],
            "--ros-args",
            "--log-level",
            "WARN",
        ],
        remappings=[
            # 2D 定位：与建图同源的水平 LaserScan（特征空间一致）
            ("scan_1", "/scan_1"),
            ("scan_2", "/scan_2"),
            ("odom", "/odom"),
            ("imu", "/imu"),
        ],
    )

    # ===== 占据栅格地图发布 =====
    # 注意：源码核实 cartographer_occupancy_grid_node 只接受 5 个 flag
    # (resolution/publish_period_sec/include_frozen_submaps/include_unfrozen_submaps/
    #  occupancy_grid_topic)，min_z/max_z/z_voxel_size/trajectory_id 均无效，已移除。
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "resolution": 0.05,
                "publish_period_sec": 1.0,
                # 纯定位：只显示 pbstream 中冻结的地图
                "include_frozen_submaps": True,
                # 不把定位过程中产生的活动 submap 画进 /map
                "include_unfrozen_submaps": False,
            }
        ],
    )

    return LaunchDescription(
        [
            LogInfo(msg=["Cartographer 3D Localization（独立重启单元，与 nav2 解耦）"]),
            *declared_arguments,
            TimerAction(period=0.0, actions=[cartographer_node]),
            TimerAction(period=1.5, actions=[occupancy_grid_node]),
        ]
    )
