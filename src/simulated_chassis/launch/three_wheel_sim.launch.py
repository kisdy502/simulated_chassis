#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_name = "simulated_chassis"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    # 键盘遥控默认关闭：prefix='xterm -e' 让 teleop 脱离 launch 的进程组，
    # launch 退出（含 Ctrl+C 优雅退出）杀不掉它，每次跑仿真都会残留一个
    # xterm+teleop 孤儿挂在 systemd 下（日常遥控已由 gamepad 替代）。
    # 需要时 start_teleop:=true，用完记得关掉那个 xterm 窗口。
    start_teleop_arg = DeclareLaunchArgument(
        "start_teleop", default_value="false",
        description="是否弹出 xterm 键盘遥控窗口（默认 false，用手柄）",
    )

    # 机器人名称（统一修改）
    robot_name = 'three_wheel_agv'

    # 本包默认机器人 = 2D 雷达版（左前/右后斜对角、底盘三角切角低位安装）。
    # 3D 仿真时：本行改为 three_wheel_chassis_3d.xacro，
    # 并把下方 bridge 的 arguments/remappings 换成 3D 版（见 bridge 处注释）。
    xacro_path = os.path.join(pkg_share, "urdf", "three_wheel_chassis_2d.xacro")
    world_path = os.path.join(pkg_share, "world", "world_m.sdf")
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

    # Gazebo 将 URDF 中的 package://simulated_chassis/... 转换成
    # model://simulated_chassis/...，因此搜索根必须包含 pkg_share 的父目录；
    # pkg_share/models 则继续供 world 中的 model://triangular_prism 使用。
    set_resource_path = SetEnvironmentVariable(
        "IGN_GAZEBO_RESOURCE_PATH",
        [
            EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
            ":",
            os.path.dirname(pkg_share),
            ":",
            os.path.join(pkg_share, "models"),
        ],
    )

    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", world_path],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_ign_gazebo",
        executable="create",
        arguments=[
            "-name", robot_name,
            "-topic", "/robot_description",
            "-x", "0.0", "-y", "0.0", "-z", "0.01",
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

    # ===== 传感器话题桥接（2D 雷达版） =====
    # gpu_lidar 单垂直波束，基础话题即 LaserScan，无点云链路；
    # Cartographer/Nav2 只消费 /scan_1 /scan_2。
    # 3D 仿真时换成：
    #   arguments=[
    #       '/front_lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #       '/rear_lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
    #       '/front_lidar/point_cloud@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    #       '/rear_lidar/point_cloud@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
    #       '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
    #       '/world/test_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
    #   ],
    #   并在 remappings 中追加：
    #       ('/front_lidar/point_cloud/points', '/points2_1'),
    #       ('/rear_lidar/point_cloud/points', '/points2_2'),
    #       同时两条 LaserScan 的 remap 源改为 '/front_lidar/point_cloud'、'/rear_lidar/point_cloud'。
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/front_lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/rear_lidar/scan@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            '/world/test_world/clock@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            (f'/model/{robot_name}/odometry', '/odom'),
            # 注意：不桥接 /model/{name}/tf → /tf！odom→base_footprint TF 由
            # 三舵轮控制器发布（publish_tf: true，three_wheel_controllers.yaml）。
            # 桥接会造成 TF 双发布者打架（Ignition 真值 vs 控制器轮式里程计），
            # TF 在两个值间跳动 → 遥控时机器人来回抖动的根因。
            ('/world/test_world/clock', '/clock'),
            ('/front_lidar/scan', '/scan_1'),
            ('/rear_lidar/scan', '/scan_2'),
        ],
        output='screen'
    )

    teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twistkeyboard',
        prefix='xterm -e',  # 在独立终端中运行（launch 退出杀不掉，见 start_teleop_arg 注释）
        condition=IfCondition(LaunchConfiguration('start_teleop')),
        remappings=[
            ('/cmd_vel', '/three_wheel_base_controller/cmd_vel'),  # 重映射
        ],

        output='screen',  # 输出会显示在启动launch的终端中
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

    # 历史兼容节点（当前不启动）：控制器已直接发布标准 /odom，
    # /three_wheel_base_controller/odom 已无发布者，不再需要二次转发。
    odom_relay_node = Node(
        package="simulated_chassis",
        executable="odom_relay_node",
        output="screen",
        parameters=[
            {'input_topic': '/three_wheel_base_controller/odom'},
            {'output_topic': '/odom'},
            {'publish_tf': False},  # 避免重复；odom TF 由 Cartographer provide_odom_frame 发布
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_teleop_arg,
        set_plugin_path,
        set_resource_path,
        set_software_render,
        robot_state_pub,
        ign_gazebo,
        spawn_after_gazebo,
        controller_spawners,
        bridge,
        teleop, ##用游戏手柄替代键盘
        joy_node,
        gamepad_teleop_node,
        # odom_relay_node,  # 控制器已直接发布 /odom
    ])
