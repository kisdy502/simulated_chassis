#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    
    # 机器人名称（统一修改）
    robot_name = 'three_wheel_agv'

    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis.xacro")
    world_path = os.path.join(pkg_share, "world", "world_sm.sdf")
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path])
    }

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )
    
    set_plugin_path = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )

    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", world_path],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.0",
        ],
        output="screen",
    )

    spawn_after_gazebo = RegisterEventHandler(
        OnProcessStart(
            target_action=ign_gazebo,
            on_start=[TimerAction(period=3.0, actions=[spawn_robot])],
        )
    )

    controller_spawners = TimerAction(
        period=6.0,
        actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
            Node(package="controller_manager", executable="spawner",
                 arguments=["three_wheel_base_controller", "--controller-manager", "/controller_manager"]),
        ],
    )
    
    # ============ 桥接（只改这里） ============
    # bridge = Node(
    #     package='ros_ign_bridge',
    #     executable='parameter_bridge',
    #     arguments=[
    #     # 3D 点云 - 注意是 /points 子话题
    #     '/lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #     # 如果需要 2D LaserScan 也桥接
    #     '/lidar/point_cloud@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
    #     '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
    #     # 把 Gazebo 的 /world/test_world/clock 桥接到 ROS2 的 /clock
    #     #'/world/test_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
    #     '/world/test_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
    #     ],
    #     output='screen'
    # )
    
    bridge = Node(
        package='ros_gz_bridge',  # 改包名
        executable='parameter_bridge',
        arguments=[
            '/lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/lidar/point_cloud@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/test_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',  # 用 gz.msgs
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            (f'/model/{robot_name}/odometry', '/odom'),
            (f'/model/{robot_name}/tf', '/tf'),
            ('/world/test_world/clock', '/clock'),  # ✅ 重映射到 /clock
            ('/lidar/point_cloud/points','points2')
        ],
        output='screen'
    )
    
    # teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twistkeyboard',
    #     prefix='xterm -e',  # 在独立终端中运行
    #     remappings=[
    #         ('/cmd_vel', '/three_wheel_base_controller/cmd_vel'),  # 重映射
    #     ],

    #     output='screen',  # 输出会显示在启动launch的终端中
    # )
    
    # joy 手柄驱动
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }],
    )

    # Gamepad 遥控节点
    gamepad_teleop_node = Node(
        package='simulated_chassis',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/three_wheel_base_controller/cmd_vel',
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            'watchdog_timeout': 0.8,   # 摇杆断连0.8秒后停车，松手主动发停不会被误杀
        }]
    )
    
    # 里程计中继：控制器发布 /three_wheel_base_controller/odom，转发到 /odom
    odom_relay_node = Node(
        package="simulated_chassis",
        executable="odom_relay_node",
        output="screen",
        parameters=[
            {'input_topic': '/three_wheel_base_controller/odom'},
            {'output_topic': '/odom'},
            {'publish_tf': False},  # TF 由控制器 enable_odom_tf 发布
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        set_plugin_path,
        set_software_render,
        robot_state_pub,
        ign_gazebo,
        spawn_after_gazebo,
        controller_spawners,
        bridge,
        # teleop, ##用游戏手柄替代键盘
        joy_node,
        gamepad_teleop_node,
        odom_relay_node,
    ])