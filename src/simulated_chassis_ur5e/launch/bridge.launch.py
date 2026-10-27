"""Simulation profile: existing AGV bridge plus the assembled arm runtime."""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import ExecutableInPackage
from ament_index_python.packages import get_package_share_directory as share
from simulated_chassis_ur5e.configuration import path


def generate_launch_description():
    defaults = {'agv_id': 'AGV001', 'port': '9090', 'address': '0.0.0.0', 'maps_dir': 'maps',
        'pbstream_file': '', 'manage_navigation': 'true', 'back_up_max_heading_error_deg': '20.0',
        'rviz': 'false', 'allow_fault_injection': 'false',
        'slam_algorithm': 'cartographer', 'require_battery_state': 'true',
        'mesh_server': 'true', 'mesh_server_port': '8090', 'enable_map_3d': 'true', 'map_3d_port': '8089',
        'mounting_file': path('config', 'mounting.yaml'),
        'camera_file': path('config', 'wrist_camera.yaml'),
        'teach_points_file': '~/.ros/arm_manager/mobile_ur5e_teach_points.json'}
    bridge_args = {key: LaunchConfiguration(key) for key in (
        'agv_id', 'port', 'address', 'maps_dir', 'pbstream_file', 'manage_navigation',
        'back_up_max_heading_error_deg', 'slam_algorithm', 'enable_map_3d', 'map_3d_port', 'require_battery_state')}
    bridge_args.update({'robot_package': 'simulated_chassis', 'use_sim_time': 'true',
        'navigation_use_rviz': 'false',
        # F1.3:放行 robot_state_publisher 参数(上位机 /api/v1/robot/urdf 读 robot_description)
        'params_glob': "['robot_description', '/agv_nav_server/*', '/robot_state_publisher/*']",
        'topics_sub_glob': "['/odom', '/tf', '/tf_static', '/map', '/plan', '/local_plan', '/joint_states', '/agv/status', '/scan_1', '/scan_2', '/agv/pose', '/arm_manager/status', '/wrist_camera/image_raw', '/wrist_camera/camera_info']",
        'services_glob': "['/rosapi/*', '/agv/set_control', '/agv/set_charging', '/agv/get_map', '/agv/load_map', '/agv/list_maps', '/agv/start_mapping', '/agv/save_map', '/agv/relocalize', '/agv/arm_move', '/arm_manager/save_teach_point', '/arm_manager/list_teach_points', '/arm_manager/delete_teach_point', '/arm_manager/cancel', '/arm_manager/reset_fault', '/wrist_camera/take_photo']"})
    declarations = [DeclareLaunchArgument(key, default_value=value, choices=['true', 'false'])
                    if key in ('manage_navigation', 'mesh_server', 'enable_map_3d')
                    else DeclareLaunchArgument(key, default_value=value)
                    for key, value in defaults.items()]
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share('agv_bridge_v2'), 'launch', 'agv_rosbridge.launch.py')),
            launch_arguments=bridge_args.items()),
        # U2/F1.4:URDF mesh 静态服务(8090)。UR5e 的 mesh 在系统包 ur_description
        # (package://ur_description/meshes/...),按 ament 索引直接命中;
        # 前端配置 VITE_URDF_MESH_BASE=http://<robot>:8090
        ExecuteProcess(
            cmd=[ExecutableInPackage(package='agv_bridge_v2', executable='urdf_mesh_server'),
                 '--port', LaunchConfiguration('mesh_server_port')],
            condition=IfCondition(LaunchConfiguration('mesh_server')), output='screen'),
        # The generic bridge owns this relay only when it manages Nav2.
        Node(package='simulated_chassis', executable='cmd_vel_relay_node', name='mobile_cmd_vel_relay',
             parameters=[{'input_topic': '/cmd_vel', 'output_topic': '/three_wheel_base_controller/cmd_vel',
                          'use_sim_time': True}], condition=UnlessCondition(LaunchConfiguration('manage_navigation'))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(path('launch', 'arm_runtime.launch.py')),
            launch_arguments={key: LaunchConfiguration(key) for key in (
                'rviz', 'mounting_file', 'camera_file', 'allow_fault_injection', 'teach_points_file')}.items()),
    ])
