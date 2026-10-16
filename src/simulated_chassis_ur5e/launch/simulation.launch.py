"""One physics world/model/manager. No MoveIt or business nodes here."""
import os
import tempfile
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory as share
from simulated_chassis_ur5e.configuration import path, merged_controllers, robot_description


def setup(context):
    # Materialize only the merged controller YAML and expanded model, not copies of chassis tuning.
    temporary = tempfile.TemporaryDirectory(prefix='three-wheel-ur5e-')
    controllers = os.path.join(temporary.name, 'controllers.yaml')
    description = os.path.join(temporary.name, 'robot.urdf')
    with open(controllers, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(merged_controllers(), stream)
    with open(description, 'w', encoding='utf-8') as stream:
        stream.write(robot_description(controllers, LaunchConfiguration('mounting_file').perform(context))['robot_description'])
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda context: temporary.cleanup())]))
    return [cleanup, IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share('simulated_chassis'), 'launch', 'three_wheel_sim.launch.py')),
        launch_arguments={
            'description_file': description, 'extra_controllers': 'joint_trajectory_controller',
            'extra_resource_path': os.path.dirname(share('ur_description')),
            'gui': LaunchConfiguration('gui'), 'use_sim_time': 'true',
            'start_gamepad': LaunchConfiguration('start_gamepad'), 'start_teleop': 'false',
            'include_cmd_vel_relay': 'false',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_gamepad', default_value='false'),
        DeclareLaunchArgument('mounting_file', default_value=path('config', 'mounting.yaml')),
        OpaqueFunction(function=setup),
    ])
