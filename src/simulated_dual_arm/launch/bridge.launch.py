#!/usr/bin/env python3
"""双臂整机 bridge 入口:通用 bridge + 上装仿真节点 + URDF mesh 静态服务。

用法:
  # 终端一(仿真):
  ros2 launch simulated_dual_arm simulation.launch.py gui:=true start_gamepad:=true
  # 终端二(bridge):
  ros2 launch simulated_dual_arm bridge.launch.py slam_algorithm:=lightning

F1 契约要点(见 nav-deployer 需求文档):
  - /joint_states 含舵轮(steer)+ 双臂(arm_left/right_*)+ 腰(waist_*)全部关节;
  - /agv/arm_move 服务由 dual_arm_sim_node 提供;
  - robot_description 参数经 /robot_state_publisher 暴露,params_glob 已放行;
  - mesh 服务默认 8090,前端 VITE_URDF_MESH_BASE=http://<host>:8090。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory as share


def generate_launch_description():
    defaults = {'agv_id': 'AGV001', 'port': '9090', 'address': '0.0.0.0', 'maps_dir': 'maps',
                'pbstream_file': '', 'manage_navigation': 'true',
                'back_up_max_heading_error_deg': '20.0', 'rviz': 'false',
                'slam_algorithm': 'cartographer',
                'mesh_server': 'true', 'mesh_server_port': '8090'}
    bridge_args = {key: LaunchConfiguration(key) for key in (
        'agv_id', 'port', 'address', 'maps_dir', 'pbstream_file', 'manage_navigation',
        'back_up_max_heading_error_deg', 'slam_algorithm')}
    bridge_args.update({'robot_package': 'simulated_chassis', 'use_sim_time': 'true',
                        'navigation_use_rviz': 'false',
                        'topics_sub_glob': "['/odom', '/tf', '/tf_static', '/map', '/plan', "
                                           "'/local_plan', '/joint_states', '/agv/status', "
                                           "'/scan_1', '/scan_2', '/agv/pose', '/tracked_pose']",
                        'services_glob': "['/rosapi/*', '/agv/set_control', '/agv/get_map', "
                                         "'/agv/load_map', '/agv/list_maps', '/agv/start_mapping', "
                                         "'/agv/save_map', '/agv/relocalize', '/agv/arm_move']",
                        'params_glob': "['/agv_nav_server/*', '/robot_state_publisher/*']"})
    declarations = [DeclareLaunchArgument(key, default_value=value,
                                          choices=['true', 'false'])
                    if key in ('manage_navigation', 'mesh_server')
                    else DeclareLaunchArgument(key, default_value=value)
                    for key, value in defaults.items()]
    return LaunchDescription(declarations + [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share('agv_bridge_v2'), 'launch', 'agv_rosbridge.launch.py')),
            launch_arguments=bridge_args.items()),
        # F1:上装节点(/joint_states 合并 + /agv/arm_move)
        Node(package='simulated_dual_arm', executable='dual_arm_sim_node.py',
             name='dual_arm_sim_node', output='screen',
             parameters=[{'use_sim_time': True}]),
        # F1.4/F3:URDF mesh 静态服务(前端 VITE_URDF_MESH_BASE 指向它)
        ExecuteProcess(
            cmd=['python3', os.path.join(share('simulated_dual_arm'), 'scripts', 'urdf_mesh_server.py'),
                 '--port', LaunchConfiguration('mesh_server_port')],
            condition=IfCondition(LaunchConfiguration('mesh_server')), output='screen'),
    ])
