"""Official UR5e model; mock hardware or Humble/Fortress physics."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def load_yaml(path):
    with open(path, encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def setup(context):
    simulation = LaunchConfiguration("mode").perform(context) == "gazebo"
    share = get_package_share_directory("simulated_ur5e")
    description_share = get_package_share_directory("ur_description")
    moveit_share = get_package_share_directory("ur_moveit_config")
    controllers = os.path.join(share, "config", "controllers.yaml")
    mappings = {
        "name": "ur", "ur_type": "ur5e", "prefix": "", "safety_limits": "true",
        "robot_ip": "0.0.0.0",
        "use_fake_hardware": "false" if simulation else "true",
        "sim_ignition": "true" if simulation else "false",
        "simulation_controllers": controllers,
    }
    description = {"robot_description": xacro.process_file(
        os.path.join(description_share, "urdf", "ur.urdf.xacro"),
        mappings=mappings).toxml()}
    semantic = {"robot_description_semantic": xacro.process_file(
        os.path.join(moveit_share, "srdf", "ur.srdf.xacro"),
        mappings={"name": "ur", "prefix": ""}).toxml()}
    ompl = {
        "planning_plugin": "ompl_interface/OMPLPlanner",
        "request_adapters": "default_planner_request_adapters/AddTimeOptimalParameterization "
                            "default_planner_request_adapters/FixWorkspaceBounds "
                            "default_planner_request_adapters/FixStartStateBounds "
                            "default_planner_request_adapters/FixStartStateCollision "
                            "default_planner_request_adapters/FixStartStatePathConstraints",
        "start_state_max_bounds_error": 0.1,
    }
    ompl.update(load_yaml(os.path.join(moveit_share, "config", "ompl_planning.yaml")))
    kinematics = load_yaml(os.path.join(moveit_share, "config", "kinematics.yaml"))
    # Official Humble configuration is a ROS parameter file with a wildcard node.
    if "/**" in kinematics:
        kinematics = kinematics["/**"]["ros__parameters"]["robot_description_kinematics"]
    parameters = [description, semantic,
        {"robot_description_kinematics": kinematics},
        {"robot_description_planning": load_yaml(os.path.join(moveit_share, "config", "joint_limits.yaml"))},
        {"planning_pipelines": ["ompl"], "default_planning_pipeline": "ompl", "ompl": ompl},
        load_yaml(os.path.join(share, "config", "moveit_controllers.yaml")),
        {"use_sim_time": simulation, "moveit_manage_controllers": False,
         "publish_robot_description_semantic": True,
         "publish_planning_scene": True, "publish_geometry_updates": True,
         "publish_state_updates": True, "publish_transforms_updates": True,
         "trajectory_execution.allowed_start_tolerance": 0.01}]
    actions = []
    if simulation:
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ur_simulation_gz"), "launch", "ur_sim_control.launch.py")),
            launch_arguments={
                "ur_type": "ur5e", "runtime_config_package": "simulated_ur5e",
                "controllers_file": "controllers.yaml", "initial_joint_controller": "joint_trajectory_controller",
                "launch_rviz": "false", "gazebo_gui": LaunchConfiguration("gui"),
                "world_file": os.path.join(share, "worlds", "ur5e_world.sdf"),
            }.items()))
    else:
        actions.extend([
            Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[description]),
            Node(package="controller_manager", executable="ros2_control_node", parameters=[description, controllers], output="screen"),
            Node(package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster", "-c", "/controller_manager"]),
            Node(package="controller_manager", executable="spawner", arguments=["joint_trajectory_controller", "-c", "/controller_manager"]),
        ])
    actions.extend([
        Node(package="moveit_ros_move_group", executable="move_group", parameters=parameters, output="screen"),
        Node(package="rviz2", executable="rviz2", name="rviz2_ur5e", parameters=parameters,
             arguments=["-d", os.path.join(moveit_share, "rviz", "view_robot.rviz")],
             condition=IfCondition(LaunchConfiguration("rviz")), output="screen"),
    ])
    if LaunchConfiguration('include_arm_manager').perform(context).lower() == 'true':
        manager_share = get_package_share_directory('arm_manager')
        actions.append(Node(package='arm_manager', executable='arm_manager_node', name='arm_manager',
            output='screen', parameters=[os.path.join(manager_share, 'config', 'ur5e.yaml'), {
                'use_sim_time': simulation, 'simulation_mode': True,
                'allow_fault_injection': LaunchConfiguration('allow_fault_injection').perform(context).lower() == 'true',
                'teach_points_file': LaunchConfiguration('teach_points_file').perform(context),
            }]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="gazebo", choices=["gazebo", "mock"]),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument('include_arm_manager', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('allow_fault_injection', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('teach_points_file', default_value='~/.ros/arm_manager/ur5e_teach_points.json'),
        OpaqueFunction(function=setup),
    ])
