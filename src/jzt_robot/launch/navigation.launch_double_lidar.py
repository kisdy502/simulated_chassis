#!/usr/bin/env python3
"""
Nav2 导航 + Cartographer定位 + RViz 可视化启动文件

使用方法:
    # 启动示例（默认参数）
    ros2 launch jzt_robot navigation.launch.py

    # 指定地图和配置
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/jz_map.pbstream

    # 阿克曼底盘 (默认)
    ros2 launch jzt_robot navigation.launch.py \
        cmd_topic:=/ackermann_steering_controller/reference_unstamped

    # 差速底盘 / 麦克纳姆底盘
    ros2 launch jzt_robot navigation.launch.py \
        cmd_topic:=/cmd_vel

    # 真机调试
    ros2 launch jzt_robot navigation.launch.py \
        use_sim_time:=false
        
    # 完整
    ros2 launch jzt_robot navigation.launch.py \
        pbstream_file:=/home/kisdy/maps/jz_map.pbstream \
        cmd_topic:=/ackermann_steering_controller/reference_unstamped \
        use_sim_time:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, LogInfo, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
# import logging
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
# logging.getLogger().setLevel(logging.DEBUG)

def generate_launch_description():
    pkg_share = get_package_share_directory('jzt_robot')
    rviz_config = os.path.join(pkg_share, 'rviz', 'nav2_double_lidar.rviz')
    # default_params_file = os.path.join(pkg_share, 'param', 'nav2_params_mppi_cartographer_mecanum.yaml')
    # nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(pkg_share,'param','nav2_params_mppi_cartographer_mecanum.yaml'))

    cartographer_config_dir = os.path.join(pkg_share, 'config')
    
    # 启动参数
    declared_arguments = [
        # `DeclareLaunchArgument` is a function used in the Python Launch API for ROS 2 to declare
        # launch arguments. Launch arguments are parameters that can be passed to a launch file when
        # it is executed.
        DeclareLaunchArgument(
            'pbstream_file',
            default_value='/home/kisdy/maps/jz_map.pbstream',
            description='Cartographer pbstream地图文件'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='localization_2d_double_lidar.lua',
            description='Cartographer Lua配置文件，双雷达导航定位用localization_2d_double_lidar.lua'
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value='/home/kisdy/projects/agv_localization_ws/install/jzt_robot/share/jzt_robot/param/nav2_params_mppi_cartographer_ackermann_double_lidar.yaml',
            description='Full path to Nav2 param file to load'
        ),
        # 新增：遥控器控制的速度话题
        DeclareLaunchArgument(
            'cmd_topic',
            default_value='/ackermann_steering_controller/reference_unstamped',
            description='手柄遥控器发布的速度话题 (阿克曼: /ackermann_steering_controller/reference_unstamped, 差速/麦克纳姆: /cmd_vel)'
        ),

        # 新增：是否使用仿真时间
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='使用 /clock 话题作为时间源（仿真为 true，真机为 false）'
        ),
    ]
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    

    # Cartographer 定位节点
    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', LaunchConfiguration('configuration_basename'),
            '-load_state_filename', LaunchConfiguration('pbstream_file'),
            # '-start_trajectory_with_default_topics', 'false',
            '--ros-args',
            '--log-level', 'WARN',          # 只显示 ERROR 和 FATAL
        ],
        remappings=[
            ('scan_1', '/scan_front'),      # 第一个雷达：/scan_1 → /scan_front
            ('scan_2', '/scan_rear'),     # 第二个雷达：/scan_2 → /scan_rear
            ('odom', '/odom'),
            ('imu', '/imu'),  # 添加这行！
        ],
    )
    
    # 占据栅格地图发布
    occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='occupancy_grid_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'resolution': 0.05,
            'publish_period_sec': 1.0,
            'frame_id': 'map',  # 添加 frame_id
        }],
    )
    
    # Nav2 导航 (只用 navigation_launch.py，不需要 bringup 的 map_server/AMCL)
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'params_file': LaunchConfiguration('nav2_params_file'),
            'use_sim_time': use_sim_time,
            'autostart': 'true',
        }.items(),
    )
    
    # RViz 可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # joy 手柄驱动
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # Gamepad 遥控节点
    gamepad_teleop_node = Node(
        package='jzt_robot',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': LaunchConfiguration('cmd_topic'),
            'use_sim_time': use_sim_time,
            'watchdog_timeout': 0.8,   # 摇杆断连0.8秒后停车，松手主动发停不会被误杀
        }]
    )
    
    cmd_vel_mux_node = Node(
        package='jzt_robot',
        executable='cmd_vel_mux_node',
        name='cmd_vel_mux_node',
        output='screen',        
        parameters=[{
            'output_topic': '/ackermann_steering_controller/reference_unstamped',
            'timeout': 0.5,
            'default_source': 'nav2',
        }]
    )

    
    ackermannVerifier_node = Node(
        package='jzt_robot',
        executable='ackermann_verifier_node',          # 与 CMake 中 add_executable 同名
        name='ackermann_verifier',
        output='screen',
        parameters=[{
            'wheelbase': 0.8,                           # 轴距，按实际填写
            'track_width': 0.56,                        # 轮距 (m)
            'max_steering_angle': 0.5236,               # 最大转向角 (rad)
            'steering_joint': 'front_left_steer_joint', # 转向关节名（如果有 /joint_states）
            'right_steer_joint': 'front_right_steer_joint',  #精确模型左右都要
            'stop_timeout': 0.8,                        # 判定停止的超时秒数
            'max_stop_distance_error': 0.10,            # 最大允许距离误差 (m)
            'max_stop_heading_error': 5.0,              # 最大允许角度误差 (°)
            'max_residual_speed': 0.02,                 # 最大残余速度 (m/s)
            'use_sim_time': use_sim_time,
        }]
    )
    
    docking_container = ComposableNodeContainer(
        name='docking_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='jzt_robot',
                plugin='jzt_robot::DockingController',
                name='docking_controller',
                parameters=[{  # 传入参数
                    'use_sim_time': use_sim_time,
                    'controller_frequency': 50.0,
                    'min_turning_radius': 1.40,
                    'max_linear_vel': 0.3,
                    'max_angular_vel': 0.5,
                    'linear_kp': 1.0,
                    'angular_kp': 2.0,
                    'approach_linear_kp': 1.5,
                    'approach_angular_kp': 3.0,
                    'slowdown_distance': 0.5,
                    'final_approach_distance': 0.15,
                    'max_linear_accel': 1.0,
                    'max_angular_accel': 2.0,
                }],
                remappings=[
                    ('/odom', '/odom'),
                    ('/cmd_vel_docking', '/cmd_vel_docking'),
                ],
            ),
        ],
        output='screen',
    )



    return LaunchDescription([
        LogInfo(msg=['==========================================']),
        LogInfo(msg=['Nav2 导航模式启动']),
        LogInfo(msg=['地图: $(var pbstream_file)']),
        LogInfo(msg=['==========================================']),

        *declared_arguments,

        TimerAction(period=0.2, actions=[cartographer_node]),
        TimerAction(period=3.0, actions=[occupancy_grid_node]),
        #  TimerAction(period=4.0, actions=[docking_container]),
        TimerAction(period=6.0, actions=[nav2_launch]),
        TimerAction(period=9.0, actions=[joy_node]),
        TimerAction(period=11.0, actions=[gamepad_teleop_node]),
        TimerAction(period=13.0, actions=[rviz_node]),
        # TimerAction(period=14.0, actions=[ackermannVerifier_node]),
        # TimerAction(period=15.0, actions=[cmd_vel_mux_node]),
       
        LogInfo(msg=['导航节点 + 手柄遥控 + RViz 已启动']),
        LogInfo(msg=['在RViz中设置2D Goal启动自主导航，或使用手柄手动控制']),
    ])
