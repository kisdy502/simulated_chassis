"""Build one combined model and derive prefixed MoveIt parameters from upstream."""
import copy
import math
import os
import re
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory as share


def path(*parts):
    return os.path.join(share('simulated_chassis_ur5e'), *parts)


def read_yaml(filename):
    with open(filename, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def gripper_config(filename=None):
    config = read_yaml(filename or path('config', 'gripper.yaml'))
    for key in ('max_width', 'initial_width', 'finger_speed', 'command_duration'):
        if isinstance(config[key], bool) or not isinstance(config[key], (int, float)) or not math.isfinite(config[key]):
            raise ValueError(f'Invalid gripper {key}')
    if not (0 < config['max_width'] <= 0.08 and 0 <= config['initial_width'] <= config['max_width']
            and config['finger_speed'] > 0 and config['command_duration'] > 0):
        raise ValueError('Invalid gripper limits: max_width must be within (0, 0.08] m')
    return config


def validate_tool(tool_type):
    if tool_type not in ('gripper', 'camera', 'none'):
        raise ValueError(f'Unknown tool_type: {tool_type}')


def merged_controllers(tool_type='gripper'):
    validate_tool(tool_type)
    result = read_yaml(os.path.join(share('simulated_chassis'), 'param', 'three_wheel_controllers.yaml'))
    arm = read_yaml(path('config', 'controllers.yaml'))
    result['controller_manager']['ros__parameters'].update(arm.pop('controller_manager')['ros__parameters'])
    result.update(arm)
    if tool_type == 'gripper':
        gripper = read_yaml(path('config', 'gripper_controllers.yaml'))
        result['controller_manager']['ros__parameters'].update(gripper.pop('controller_manager')['ros__parameters'])
        result.update(gripper)
    return result


def robot_description(controllers_file=None, mounting_file=None, package_meshes=False, camera_file=None,
                      tool_type='gripper', gripper_file=None):
    """组合模型描述。

    始终以 sim_ignition=true 生成:官方 ur 宏把「硬件插件选择」与「mesh 路径
    形式」绑在同一开关上(sim_ignition=false 会选用真机驱动
    ur_robot_driver/URPositionHardwareInterface,gz 侧加载失败、机械臂无硬件
    接口)。robot_state_publisher 用的 package:// mesh 版通过 package_meshes
    对 file:// 前缀做受控改写获得(上位机前端按 VITE_URDF_MESH_BASE 重写),
    硬件块保持 IgnitionSystem 不变。"""
    validate_tool(tool_type)
    if tool_type == 'gripper':
        gripper_config(gripper_file)
    mappings = {'controllers_file': controllers_file or path('config', 'controllers.yaml'),
                'mounting_file': mounting_file or path('config', 'mounting.yaml'),
                'sim_ignition': 'true',
                'camera_file': camera_file or path('config', 'wrist_camera.yaml'),
                'tool_type': tool_type, 'gripper_file': gripper_file or path('config', 'gripper.yaml')}
    xml = xacro.process_file(path('urdf', 'robot.urdf.xacro'), mappings=mappings).toxml()
    if package_meshes:
        xml = re.sub(r'file://[^"\'<>]*?/share/([\w.-]+)/', r'package://\1/', xml)
    return {'robot_description': xml}


def moveit_parameters(tool_type='gripper', gripper_file=None):
    validate_tool(tool_type)
    upstream = share('ur_moveit_config')
    joints = ['ur5e_' + name for name in read_yaml(os.path.join(share('simulated_ur5e'), 'config', 'initial_positions.yaml'))]
    kinematics = read_yaml(os.path.join(upstream, 'config', 'kinematics.yaml'))
    if '/**' in kinematics:
        kinematics = kinematics['/**']['ros__parameters']['robot_description_kinematics']
    kinematics = {'ur5e_ur_manipulator': copy.deepcopy(kinematics['ur_manipulator'])}
    limits = read_yaml(os.path.join(upstream, 'config', 'joint_limits.yaml'))
    limits['joint_limits'] = {'ur5e_' + key: value for key, value in limits['joint_limits'].items()}
    ompl = read_yaml(os.path.join(upstream, 'config', 'ompl_planning.yaml'))
    ompl['ur5e_ur_manipulator'] = ompl.pop('ur_manipulator')
    ompl.update({'planning_plugin': 'ompl_interface/OMPLPlanner',
        'request_adapters': 'default_planner_request_adapters/AddTimeOptimalParameterization '
                            'default_planner_request_adapters/FixWorkspaceBounds '
                            'default_planner_request_adapters/FixStartStateBounds '
                            'default_planner_request_adapters/FixStartStateCollision '
                            'default_planner_request_adapters/FixStartStatePathConstraints',
        'start_state_max_bounds_error': 0.1, 'path_tolerance': 0.01})
    semantic = xacro.process_file(path('config', 'moveit', 'robot.srdf.xacro'), mappings={
        'tool_type': tool_type, 'gripper_file': gripper_file or path('config', 'gripper.yaml')})
    # Extend the existing six-axis chain through fixed tool links, preserving its group name.
    if tool_type == 'gripper':
        for group in semantic.getElementsByTagName('group'):
            if group.getAttribute('name') == 'ur5e_ur_manipulator':
                for chain in group.getElementsByTagName('chain'):
                    chain.setAttribute('tip_link', 'ur5e_gripper_tcp')
        ompl['ur5e_gripper'] = {'planner_configs': ['RRTConnectkConfigDefault']}
        grip = gripper_config(gripper_file)
        for name in ('ur5e_gripper_left_joint', 'ur5e_gripper_right_joint'):
            limits['joint_limits'][name] = {'has_velocity_limits': True, 'max_velocity': grip['finger_speed'],
                                           'has_acceleration_limits': True, 'max_acceleration': 0.2}
    controllers = {'controller_names': ['joint_trajectory_controller'],
        'joint_trajectory_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                                        'default': True, 'joints': joints}}
    if tool_type == 'gripper':
        controllers['controller_names'].append('gripper_controller')
        controllers['gripper_controller'] = {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
            'default': True, 'joints': ['ur5e_gripper_left_joint', 'ur5e_gripper_right_joint']}
    return [robot_description(tool_type=tool_type, gripper_file=gripper_file),
        {'robot_description_semantic': semantic.toxml()},
        {'robot_description_kinematics': kinematics, 'robot_description_planning': limits},
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl},
        {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
         'moveit_simple_controller_manager': controllers},
        {'use_sim_time': True, 'moveit_manage_controllers': False,
         'publish_robot_description_semantic': True, 'publish_planning_scene': True,
         'publish_geometry_updates': True, 'publish_state_updates': True, 'publish_transforms_updates': True,
         'trajectory_execution.allowed_start_tolerance': 0.01}]


def manager_parameters(tool_type='gripper'):
    joints = ['ur5e_' + name for name in read_yaml(os.path.join(share('simulated_ur5e'), 'config', 'initial_positions.yaml'))]
    return {'group_name': 'ur5e_ur_manipulator',
            'tool_link': 'ur5e_gripper_tcp' if tool_type == 'gripper' else 'ur5e_tool0',
            'joint_names': joints, 'use_sim_time': True, 'simulation_mode': True}
