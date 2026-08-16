#!/usr/bin/env python3
"""
差速机器人仿真启动文件 (双3D雷达 + IMU)
ROS2 Humble/Jazzy + Gazebo Fortress/Garden

传感器：前3D雷达(Mid-360仿真) + 后3D雷达(Mid-360仿真) + IMU
驱动方式：ros2_control + diff_drive_controller
点云话题：/points2_1(前), /points2_2(后) 供 Cartographer 3D 建图/导航使用

ros2 launch zioneer_robot gazebo_diff_3dlidar.launch.py
"""
import os
import xml.etree.ElementTree as ET
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def get_world_name(sdf_path: str) -> str:
    tree = ET.parse(sdf_path)
    world_elem = tree.getroot().find("world")
    if world_elem is not None:
        return world_elem.get("name", "default")
    return "default"


def generate_launch_description():
    pkg_name = "zioneer_robot"
    pkg_share = get_package_share_directory(pkg_name)

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    robot_name_arg = DeclareLaunchArgument(
        "robot_name", default_value="diff_agv01", description="机器人名称"
    )

    robot_name = "diff_agv01"
    robot_name_launch = LaunchConfiguration("robot_name")
    world_path = os.path.join(pkg_share, "world", "world_m.sdf")
    world_name = get_world_name(world_path)

    xacro_path = os.path.join(pkg_share, "urdf", "diff", "robot.xacro")
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path, " robot_name:=", robot_name_launch])
    }

    set_plugin_path = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH", "/opt/ros/humble/lib"
    )
    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    # 1. robot_state_publisher
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # 2. Gazebo (Server模式)
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "", world_path],
        output="screen",
    )

    # 3. 生成机器人 (Gazebo 启动 3 秒后)
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
            target_action=gazebo,
            on_start=[TimerAction(period=3.0, actions=[spawn_robot])],
        )
    )

    # 4. ros2_control 控制器 (Gazebo 启动 6 秒后)
    controller_spawners = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
            ),
        ],
    )

    # 5. ros_gz_bridge: 双3D雷达点云 + IMU + clock → ROS2 话题
    clock_gz_topic = f"/world/{world_name}/clock"

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            # 双3D雷达点云（前+后），Ignition 在 <topic>/points 子话题发 PointCloudPacked
            '/front_lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/rear_lidar/point_cloud/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked',
            '/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU',
            f'{clock_gz_topic}@rosgraph_msgs/msg/Clock@ignition.msgs.Clock',
        ],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            (f'/model/{robot_name}/odometry', '/odom'),
            (f'/model/{robot_name}/tf', '/tf'),
            (clock_gz_topic, '/clock'),
            # Ignition 的 /xxx/points 桥接到 ROS 后，先发到 *_raw，再由本体过滤节点输出 /points2_1 /points2_2
            ('/front_lidar/point_cloud/points', '/points2_1'),
            ('/rear_lidar/point_cloud/points', '/points2_2'),
        ],
        output="screen",
    )

    # 6.1 PointCloud2 本体过滤器：剔除 360° 雷达打在机器人本体上的回波
    def make_cloud_filter_node(name, in_topic, out_topic):
        return Node(
            package="zioneer_robot",
            executable="pointcloud_footprint_filter_node",
            name=name,
            output="screen",
            parameters=[{
                "input_topic": in_topic,
                "output_topic": out_topic,
                "base_frame": "base_link",
                "box_half_x": 0.30,
                "box_half_y": 0.20,
                "box_min_z": -0.06,
                "box_max_z": 0.06,
                "margin": 0.03,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
        )

    front_cloud_filter = make_cloud_filter_node("front_cloud_filter", "/points2_1_raw", "/points2_1")
    rear_cloud_filter = make_cloud_filter_node("rear_cloud_filter", "/points2_2_raw", "/points2_2")

    # 6. 键盘遥控（需在单独终端手动运行，见下方注释）
    #    xterm 在 Docker/WSL 下可能无法弹出，改为手动启动：
    #    ros2 run teleop_twist_keyboard teleop_twist_keyboard
    # teleop = Node(
    #     package="teleop_twist_keyboard",
    #     executable="teleop_twist_keyboard",
    #     name="teleop_twistkeyboard",
    #     prefix="xterm -e",
    #     parameters=[{"stamped": False}],
    #     output="screen",
    # )

    # 7. 手柄遥控（joy 驱动 + gamepad_teleop 节点）
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'autorepeat_rate': 20.0,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    gamepad_teleop_node = Node(
        package='zioneer_robot',
        executable='gamepad_teleop_node',
        name='gamepad_teleop_node',
        output='screen',
        parameters=[{
            'cmd_topic': '/cmd_vel',
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'watchdog_timeout': 0.8,
        }],
    )

    # 里程计转发：/diff_drive_controller/odom → /odom
    odom_relay = Node(
        package="zioneer_robot",
        executable="odom_relay_node",
        name="odom_relay_node",
        output="screen",
        parameters=[
            {"input_topic": "/diff_drive_controller/odom"},
            {"output_topic": "/odom"},
            {"publish_tf": False},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # 速度转发: /cmd_vel(Twist) → /diff_drive_controller/cmd_vel(TwistStamped)
    # diff_drive_controller 需要 TwistStamped，用 twist_to_stamped_relay 转换
    cmd_vel_relay = Node(
        package="zioneer_robot",
        executable="twist_to_stamped_relay",
        name="twist_to_stamped_relay",
        output="screen",
        parameters=[
            {"input_topic": "/cmd_vel"},
            {"output_topic": "/diff_drive_controller/cmd_vel"},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_name_arg,
        set_plugin_path,
        set_software_render,
        robot_state_pub,
        gazebo,
        spawn_after_gazebo,
        controller_spawners,
        bridge,
        # front_cloud_filter,
        # rear_cloud_filter,
        # teleop,  # 键盘遥控需手动启动
        joy_node,
        gamepad_teleop_node,
        odom_relay,
        cmd_vel_relay,
    ])
