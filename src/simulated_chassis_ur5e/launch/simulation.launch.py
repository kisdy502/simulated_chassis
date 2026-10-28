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
from simulated_chassis_ur5e.configuration import path, merged_controllers, robot_description, read_yaml
from launch_ros.actions import Node


def setup(context):
    tool_type = LaunchConfiguration('tool_type').perform(context)
    gripper_file = LaunchConfiguration('gripper_file').perform(context)
    # Materialize only the merged controller YAML and expanded model, not copies of chassis tuning.
    temporary = tempfile.TemporaryDirectory(prefix='three-wheel-ur5e-')
    controllers = os.path.join(temporary.name, 'controllers.yaml')
    # 两份描述:均以 sim_ignition=true 生成(硬件块=IgnitionSystem);RSP 版仅把
    # mesh 前缀改写为 package://(上位机经 8090 拉取;gz 的 ros2_control 插件
    # 也读 RSP 参数,硬件块必须保持仿真插件,详见 configuration.robot_description)
    description = os.path.join(temporary.name, 'robot.urdf')
    description_rsp = os.path.join(temporary.name, 'robot_rsp.urdf')
    with open(controllers, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(merged_controllers(tool_type), stream)
    mounting = LaunchConfiguration('mounting_file').perform(context)
    camera_file = LaunchConfiguration('camera_file').perform(context)
    camera = read_yaml(camera_file)
    with open(description, 'w', encoding='utf-8') as stream:
        stream.write(robot_description(controllers, mounting, camera_file=camera_file,
                                      tool_type=tool_type, gripper_file=gripper_file)['robot_description'])
    with open(description_rsp, 'w', encoding='utf-8') as stream:
        stream.write(robot_description(controllers, mounting, package_meshes=True, camera_file=camera_file,
                                      tool_type=tool_type, gripper_file=gripper_file)['robot_description'])
    cleanup = RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(
        function=lambda context: temporary.cleanup())]))
    camera_nodes = []
    if tool_type == 'camera' and camera['enabled']:
        camera_nodes = [
            Node(package='ros_gz_bridge', executable='parameter_bridge', name='wrist_camera_bridge',
                 arguments=[
                     '/wrist_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
                     '/wrist_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                 remappings=[('/wrist_camera/image', '/wrist_camera/raw/image'),
                             ('/wrist_camera/camera_info', '/wrist_camera/raw/camera_info')],
                 parameters=[{'use_sim_time': True}], output='screen'),
            Node(package='simulated_chassis_ur5e', executable='wrist_camera_node',
                 parameters=[{'use_sim_time': True,
                              'output_dir': LaunchConfiguration('photos_dir')}], output='screen'),
        ]
    return camera_nodes + [cleanup, IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(share('simulated_chassis'), 'launch', 'three_wheel_sim.launch.py')),
        launch_arguments={
            'description_file': description,
            'rsp_description_file': description_rsp,
            'extra_controllers': 'joint_trajectory_controller gripper_controller' if tool_type == 'gripper' else 'joint_trajectory_controller',
            'extra_resource_path': os.path.dirname(share('ur_description')),
            'gui': LaunchConfiguration('gui'), 'use_sim_time': 'true',
            'battery_simulation': LaunchConfiguration('battery_simulation'),
            'start_gamepad': LaunchConfiguration('start_gamepad'), 'start_teleop': 'true',
            'include_cmd_vel_relay': 'true',
        }.items())]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='gripper', choices=['gripper', 'camera', 'none']),
        DeclareLaunchArgument('gripper_file', default_value=path('config', 'gripper.yaml')),
        DeclareLaunchArgument('battery_simulation', default_value='true'),
        DeclareLaunchArgument('camera_file', default_value=path('config', 'wrist_camera.yaml')),
        DeclareLaunchArgument('photos_dir', default_value='~/.ros/wrist_camera/photos'),
        DeclareLaunchArgument('gui', default_value='false'),
        DeclareLaunchArgument('start_gamepad', default_value='true'),
        DeclareLaunchArgument('mounting_file', default_value=path('config', 'mounting.yaml')),
        OpaqueFunction(function=setup),
    ])
