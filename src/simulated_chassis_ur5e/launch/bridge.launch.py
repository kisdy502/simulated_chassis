"""Simulation profile: existing AGV bridge plus the assembled arm runtime."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share
from simulated_chassis_ur5e.configuration import path


def generate_launch_description():
    defaults = {'agv_id': 'AGV001', 'port': '9090', 'address': '0.0.0.0', 'maps_dir': 'maps',
        'pbstream_file': '', 'manage_navigation': 'true', 'back_up_max_heading_error_deg': '20.0',
        'rviz': 'false', 'allow_fault_injection': 'false',
        'slam_algorithm': 'cartographer',
        'mounting_file': path('config', 'mounting.yaml'),
        'teach_points_file': '~/.ros/arm_manager/mobile_ur5e_teach_points.json'}
    bridge_args = {key: LaunchConfiguration(key) for key in (
        'agv_id', 'port', 'address', 'maps_dir', 'pbstream_file', 'manage_navigation',
        'back_up_max_heading_error_deg', 'slam_algorithm')}
    bridge_args.update({'robot_package': 'simulated_chassis', 'use_sim_time': 'true',
        'navigation_use_rviz': 'false',
        'topics_sub_glob': "['/odom', '/tf', '/tf_static', '/map', '/plan', '/local_plan', '/joint_states', '/agv/status', '/scan_1', '/scan_2', '/agv/pose', '/arm_manager/status']",
        'services_glob': "['/rosapi/*', '/agv/set_control', '/agv/get_map', '/agv/load_map', '/agv/list_maps', '/agv/start_mapping', '/agv/save_map', '/agv/relocalize', '/arm_manager/save_teach_point', '/arm_manager/list_teach_points', '/arm_manager/delete_teach_point', '/arm_manager/cancel', '/arm_manager/reset_fault']"})
    declarations = [DeclareLaunchArgument(key, default_value=value, choices=['true', 'false'])
                    if key == 'manage_navigation' else DeclareLaunchArgument(key, default_value=value)
                    for key, value in defaults.items()]
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share('agv_bridge_v2'), 'launch', 'agv_rosbridge.launch.py')),
            launch_arguments=bridge_args.items()),
        # The generic bridge owns this relay only when it manages Nav2.
        Node(package='simulated_chassis', executable='cmd_vel_relay_node', name='mobile_cmd_vel_relay',
             parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/three_wheel_base_controller/cmd_vel',
                          'use_sim_time': True}], condition=UnlessCondition(LaunchConfiguration('manage_navigation'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path('launch', 'arm_runtime.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in (
                'rviz', 'mounting_file', 'allow_fault_injection', 'teach_points_file')}.items()),
    ])
