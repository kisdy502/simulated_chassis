#!/usr/bin/env python3
"""双臂整机物理仿真入口:复用底盘仿真 launch,仅替换组合模型描述。

用法:
  ros2 launch simulated_dual_arm simulation.launch.py gui:=true start_gamepad:=true

与 simulated_chassis_ur5e 的 simulation.launch.py 同构:一个世界、一台机器人、
一个 controller_manager;上装为视觉级(无 ros2_control 关节),不需要额外控制器。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory as share
import os


def generate_launch_description():
    description = os.path.join(share('simulated_dual_arm'), 'urdf', 'robot.urdf.xacro')
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_gamepad', default_value='false',
                              description='手柄遥控(joy_node+gamepad_teleop_node);键盘遥控用 start_teleop'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share('simulated_chassis'), 'launch', 'three_wheel_sim.launch.py')),
            launch_arguments={
                'description_file': description,
                'start_gamepad': LaunchConfiguration('start_gamepad'),
                'gui': LaunchConfiguration('gui'),
                'use_sim_time': 'true',
            }.items()),
    ])
