"""Build one combined model and derive prefixed MoveIt parameters from upstream."""
import copy
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


def merged_controllers():
    result = read_yaml(os.path.join(share('simulated_chassis'), 'param', 'three_wheel_controllers.yaml'))
    arm = read_yaml(path('config', 'controllers.yaml'))
    result['controller_manager']['ros__parameters'].update(arm.pop('controller_manager')['ros__parameters'])
    result.update(arm)
    return result


def robot_description(controllers_file=None, mounting_file=None, package_meshes=False):
    """组合模型描述。

    始终以 sim_ignition=true 生成:官方 ur 宏把「硬件插件选择」与「mesh 路径
    形式」绑在同一开关上(sim_ignition=false 会选用真机驱动
    ur_robot_driver/URPositionHardwareInterface,gz 侧加载失败、机械臂无硬件
    接口)。robot_state_publisher 用的 package:// mesh 版通过 package_meshes
    对 file:// 前缀做受控改写获得(上位机前端按 VITE_URDF_MESH_BASE 重写),
    硬件块保持 IgnitionSystem 不变。"""
    mappings = {'controllers_file': controllers_file or path('config', 'controllers.yaml'),
                'mounting_file': mounting_file or path('config', 'mounting.yaml'),
                'sim_ignition': 'true'}
    xml = xacro.process_file(path('urdf', 'robot.urdf.xacro'), mappings=mappings).toxml()
    if package_meshes:
        xml = re.sub(r'file://[^"\'<>]*?/share/([\w.-]+)/', r'package://\1/', xml)
    return {'robot_description': xml}


def moveit_parameters():
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
    return [robot_description(),
        {'robot_description_semantic': xacro.process_file(path('config', 'moveit', 'robot.srdf.xacro')).toxml()},
        {'robot_description_kinematics': kinematics, 'robot_description_planning': limits},
        {'planning_pipelines': ['ompl'], 'default_planning_pipeline': 'ompl', 'ompl': ompl},
        {'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
         'moveit_simple_controller_manager': {'controller_names': ['joint_trajectory_controller'],
            'joint_trajectory_controller': {'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory',
                                            'default': True, 'joints': joints}}},
        {'use_sim_time': True, 'moveit_manage_controllers': False,
         'publish_robot_description_semantic': True, 'publish_planning_scene': True,
         'publish_geometry_updates': True, 'publish_state_updates': True, 'publish_transforms_updates': True,
         'trajectory_execution.allowed_start_tolerance': 0.01}]


def manager_parameters():
    joints = ['ur5e_' + name for name in read_yaml(os.path.join(share('simulated_ur5e'), 'config', 'initial_positions.yaml'))]
    return {'group_name': 'ur5e_ur_manipulator', 'tool_link': 'ur5e_tool0',
            'joint_names': joints, 'use_sim_time': True, 'simulation_mode': True}
