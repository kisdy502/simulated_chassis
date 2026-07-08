#!/usr/bin/env python3
"""
AGV Bridge Launch File
用于启动ROS2与Spring Boot的桥梁节点
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


def generate_launch_description():
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # 定义启动参数
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="是否使用仿真时间"
    )

    agv_id_arg = DeclareLaunchArgument(
        "agv_id", default_value="AGV001", description="AGV的唯一标识符"
    )

    springboot_host_arg = DeclareLaunchArgument(
        "springboot_host",
        default_value="127.0.0.1",
        description="Spring Boot服务器地址",
    )

    springboot_port_arg = DeclareLaunchArgument(
        "springboot_port", default_value="22777", description="Spring Boot服务器端口"
    )

    websocket_port_arg = DeclareLaunchArgument(
        "websocket_port", default_value="9090", description="WebSocket服务器端口"
    )

    imu_serial_port_arg = DeclareLaunchArgument(
        "imu_serial_port", default_value="/dev/ttyACM0", description="IMU串口设备"
    )

    imu_baud_rate_arg = DeclareLaunchArgument(
        "imu_baud_rate", default_value="115200", description="IMU波特率"
    )

    # 创建节点
    agv_bridge_node = Node(
        package="agv_bridge_v2",
        executable="agv_bridge_node",
        name="agv_bridge_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "agv_id": LaunchConfiguration("agv_id"),
                "springboot_host": LaunchConfiguration("springboot_host"),
                "springboot_port": LaunchConfiguration("springboot_port"),
                "websocket_port": LaunchConfiguration("websocket_port"),
                "imu_serial_port": LaunchConfiguration("imu_serial_port"),
                "imu_baud_rate": LaunchConfiguration("imu_baud_rate"),
            }
        ],
        # use_sim_time 是 rclcpp 内部参数，通过 --ros-args 传入最可靠
        arguments=["--ros-args", "--log-level", "info", "-p", "use_sim_time:=true"],
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/points2'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'lidar_link',      # 雷达坐标系
            'transform_tolerance': 0.01,
            'min_height': -0.1,                 # 截取高度下限
            'max_height': 0.1,                  # 截取高度上限
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,          # ~0.5°
            'scan_time': 0.1,
            'range_min': 0.2,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
            'concurrency_level': 1,
        }],
        arguments=["--ros-args", "-p", "use_sim_time:=true"],
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            agv_id_arg,
            springboot_host_arg,
            springboot_port_arg,
            websocket_port_arg,
            imu_serial_port_arg,
            imu_baud_rate_arg,
            agv_bridge_node,
            pointcloud_to_laserscan_node
        ]
    )
