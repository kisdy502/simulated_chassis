from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=PathJoinSubstitution([
            FindPackageShare('arm_manager'), 'config', 'ur5e.yaml'])),
        DeclareLaunchArgument('simulation_mode', default_value='false'),
        DeclareLaunchArgument('allow_fault_injection', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('teach_points_file', default_value='~/.ros/arm_manager/ur5e_teach_points.json'),
        Node(package='arm_manager', executable='arm_manager_node', name='arm_manager',
             output='screen', parameters=[LaunchConfiguration('config'), {
                 'simulation_mode': ParameterValue(LaunchConfiguration('simulation_mode'), value_type=bool),
                 'allow_fault_injection': ParameterValue(LaunchConfiguration('allow_fault_injection'), value_type=bool),
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'teach_points_file': LaunchConfiguration('teach_points_file'),
             }]),
    ])
