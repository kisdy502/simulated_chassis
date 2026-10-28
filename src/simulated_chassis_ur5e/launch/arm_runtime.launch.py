"""Business runtime only; never spawn a robot, clock or controller manager."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent, LogInfo
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from simulated_chassis_ur5e.configuration import path, robot_description, moveit_parameters, manager_parameters, gripper_config


def setup(context):
    tool_type = LaunchConfiguration('tool_type').perform(context)
    gripper_file = LaunchConfiguration('gripper_file').perform(context)
    parameters = moveit_parameters(tool_type, gripper_file)
    parameters[0] = robot_description(mounting_file=LaunchConfiguration('mounting_file').perform(context),
        camera_file=LaunchConfiguration('camera_file').perform(context),
        tool_type=tool_type, gripper_file=gripper_file)
    gate = Node(package='simulated_chassis_ur5e', executable='wait_for_robot',
                parameters=[parameters[0], {'timeout_sec': 90.0, 'require_gripper': tool_type == 'gripper'}], output='screen')
    runtime = [Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
                    parameters=parameters, output='screen',
                    on_exit=[EmitEvent(event=Shutdown(reason='MoveIt runtime exited'))]),
        # /agv/arm_move → JTC 薄适配(上位机机械臂控制契约;受理即答,JTC 插值执行)
        Node(package='simulated_chassis_ur5e', executable='arm_move_adapter',
             name='arm_move_adapter', output='screen',
             parameters=[path('config', 'arm_move_adapter.yaml'), parameters[0], {'use_sim_time': True}]),
        Node(package='arm_manager', executable='arm_manager_node', name='arm_manager', output='screen',
             sigterm_timeout='12.0', sigkill_timeout='3.0',
             parameters=[manager_parameters(tool_type), {
                 'allow_fault_injection': LaunchConfiguration('allow_fault_injection').perform(context) == 'true',
                 'teach_points_file': LaunchConfiguration('teach_points_file').perform(context)}]),
        Node(package='rviz2', executable='rviz2', name='rviz_mobile_ur5e', output='screen',
             parameters=parameters, arguments=['-d', path('config', 'moveit', 'mobile_ur5e.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')))]

    if tool_type == 'gripper':
        grip = gripper_config(gripper_file)
        runtime.append(Node(package='simulated_chassis_ur5e', executable='gripper_control',
            name='gripper_control', output='screen', parameters=[{
                key: float(grip[key]) for key in ('max_width', 'finger_speed', 'command_duration')},
                {'use_sim_time': True}]))

    def after_gate(event, context):
        if event.returncode == 0:
            return runtime
        return [LogInfo(msg='ERROR: Mobile UR5e readiness check failed; arm runtime will not start'),
                EmitEvent(event=Shutdown(reason='Combined robot not ready'))]

    return [RegisterEventHandler(OnProcessExit(target_action=gate, on_exit=after_gate)), gate]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='gripper', choices=['gripper', 'camera', 'none']),
        DeclareLaunchArgument('gripper_file', default_value=path('config', 'gripper.yaml')),
        DeclareLaunchArgument('camera_file', default_value=path('config', 'wrist_camera.yaml')),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('mounting_file', default_value=path('config', 'mounting.yaml')),
        DeclareLaunchArgument('allow_fault_injection', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('teach_points_file', default_value='~/.ros/arm_manager/mobile_ur5e_teach_points.json'),
        OpaqueFunction(function=setup),
    ])
