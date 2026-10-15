#!/usr/bin/env python3
"""六轴机械臂 RViz 规划演示（mock 硬件，不启动 Gazebo）

启动内容:
  - robot_state_publisher / ros2_control_node(fake_components)
  - joint_state_broadcaster + arm_controller(JTC)
  - move_group(MoveIt2 OMPL)
  - RViz(MotionPlanning 面板，拖动末端目标位姿 -> Plan & Execute)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    pkg_share = get_package_share_directory("simulated_arm")
    xacro_file = os.path.join(pkg_share, "urdf", "arm6_standalone.urdf.xacro")
    controllers_file = os.path.join(pkg_share, "config", "ros2_controllers_fake.yaml")
    rviz_file = os.path.join(pkg_share, "config", "moveit.rviz")

    robot_description = {
        "robot_description": Command(["xacro ", xacro_file, " use_ignition:=false"])
    }

    moveit_config = (
        MoveItConfigsBuilder("simulated_arm", package_name="simulated_arm")
        .robot_description(
            file_path="urdf/arm6_standalone.urdf.xacro",
            mappings={"use_ignition": "false"},
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
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--controller-manager", "/controller_manager"],
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_file],
        parameters=[
            robot_description,
            # MotionPlanning 显示插件需要 SRDF（机器人模型）和 IK 配置（拖标记求逆解）
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription(
        [
            robot_state_pub,
            control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            move_group_node,
            rviz_node,
        ]
    )
