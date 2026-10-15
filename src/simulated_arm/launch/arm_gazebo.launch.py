#!/usr/bin/env python3
"""六轴机械臂 Gazebo(Ignition Fortress) + MoveIt2 联动仿真

链路: RViz(MotionPlanning) -> move_group -> /arm_controller/follow_joint_trajectory
      -> JTC(Gazebo 内 controller_manager) -> ign_ros2_control -> Gazebo 关节

用法:
  ros2 launch simulated_arm arm_gazebo.launch.py
  gui:=false   不启动 Gazebo 图形界面（服务器模式）
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, TimerAction,
    RegisterEventHandler, SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("simulated_arm")
    xacro_file = os.path.join(pkg_share, "urdf", "arm6_standalone.urdf.xacro")
    world_path = os.path.join(pkg_share, "worlds", "arm_world.sdf")
    rviz_file = os.path.join(pkg_share, "config", "moveit.rviz")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="使用仿真时间"
    )
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true", description="是否启动 Gazebo 图形界面"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="是否启动 RViz"
    )

    robot_name = "simulated_arm"
    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " use_ignition:=true"])
    }

    moveit_config = (
        MoveItConfigsBuilder("simulated_arm", package_name="simulated_arm")
        .robot_description(
            file_path="urdf/arm6_standalone.urdf.xacro",
            mappings={"use_ignition": "true"},
        )
        .robot_description_semantic(file_path="config/simulated_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # ign_ros2_control-system 插件搜索路径（与 three_wheel_sim 一致）
    set_plugin_path = SetEnvironmentVariable(
        "IGN_GAZEBO_SYSTEM_PLUGIN_PATH",
        "/opt/ros/humble/lib"
    )

    ign_gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", world_path],
        output="screen",
    )

    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-g"], output="screen",
        condition=IfCondition(LaunchConfiguration("gui")),
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

    # Gazebo 进程内的 controller_manager（由 xacro 的 <parameters> 加载 sim 版配置）
    controller_spawners = TimerAction(
        period=6.0,
        actions=[
            Node(package="controller_manager", executable="spawner",
                 arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"]),
            Node(package="controller_manager", executable="spawner",
                 arguments=["arm_controller", "--controller-manager", "/controller_manager"]),
        ],
    )

    # 仿真时钟桥接（机械臂 spawn 后 move_group/控制器才能正确使用 sim time）
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/arm_world/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        remappings=[("/world/arm_world/clock", "/clock")],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[
            robot_description,
            # MotionPlanning 显示插件需要 SRDF 和 IK 配置
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )

    return LaunchDescription(
        [
            use_sim_time_arg,
            gui_arg,
            rviz_arg,
            set_plugin_path,
            robot_state_pub,
            clock_bridge,
            ign_gazebo,
            spawn_after_gazebo,
            controller_spawners,
            # 等 /joint_states 可用后再起 move_group，避免 CurrentStateMonitor 空转告警
            TimerAction(period=8.0, actions=[move_group_node]),
            TimerAction(period=8.5, actions=[rviz_node]),
            TimerAction(period=2.0, actions=[gazebo_gui]),
        ]
    )
