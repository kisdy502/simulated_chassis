import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world_name = LaunchConfiguration('world_name', default='nav_slam_world')
    
    # 修改为你的模型路径
    model_path = "/home/kisdy/workspace/simulated_chassis/src/simulated_chassis/world"
    
    # 机器人名称（统一修改）
    robot_name = 'three_wheel_agv'
    
    set_software_render = SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", "1")

    # Ignition Gazebo模型路径设置
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=[
            os.path.join("/opt/ros/humble", "share"),  # 注意：你用的是humble
            ":" + model_path
        ]
    )

    # ===== 生成机器人 =====
    ignition_spawn_entity = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-name', robot_name,
            '-file', PathJoinSubstitution([model_path, "robot.sdf"]),
            '-allow_renaming', 'true',
            '-x', '0',
            '-y', '0',
            '-z', '0.1',  # 稍微抬高一点避免陷入地面
        ],
    )
    
    # ===== 生成世界（如果world.sdf存在） =====
    ignition_spawn_world = Node(
        package='ros_ign_gazebo',
        executable='create',
        output='screen',
        arguments=[
            '-file', PathJoinSubstitution([model_path, "field.sdf"]),
            '-allow_renaming', 'false'
        ],
    )

    # ===== ROS2-Gazebo桥接节点（核心修改） =====
    bridge_ign2ros2 = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='bridge_node',
        arguments=[
            # 速度控制（保持不变）
            '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            
            # 3D雷达 - 修改为你的雷达话题（通常是PointCloud2）
            '/lidar/point_cloud@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked',
            
            # IMU话题
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
            
            # 里程计（修改机器人名前缀）
            f'/model/{robot_name}/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            
            # TF（修改机器人名前缀）
            f'/model/{robot_name}/tf@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            
            # 如果需要摄像头，取消注释并修改
            # '/camera_front/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            # '/camera_front/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            (f'/model/{robot_name}/odometry', '/odom'),
            (f'/model/{robot_name}/tf', '/tf'),
        ],
    )

    # ===== 静态TF变换（根据你的机器人尺寸调整） =====
    
    # base_link到base_footprint（调整高度）
    base_link2base_footprint_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "-0.075", "0", "0", "0", "base_link", "base_footprint"]
        # 你的base_link高度是0.075（z坐标一半）
    )

    # 3D雷达到base_link的TF（根据你的雷达安装位置）
    lidar2base_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0.12", "0", "0", "0", "lidar_link", "base_link"]
        # 你的雷达在base_link上方0.12m
    )

    # 如果需要添加各转向关节的TF（可视化用）
    steering_tfs = []
    for wheel in ['front', 'left', 'right']:
        steering_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=[
                "0", "0", "0", "0", "0", "0",
                f"wheel_{wheel}_steering_link", f"wheel_{wheel}_wheel_link"
            ]
        )
        steering_tfs.append(steering_tf)

    # ===== 键盘控制 =====
    # teleop = Node(
    #     package='teleop_twist_keyboard',
    #     executable='teleop_twist_keyboard',
    #     name='teleop_twistkeyboard',
    #     prefix="xterm -e",
    # )

    # ===== 机器人状态发布器（从SDF转URDF） =====
    sdf_path = os.path.join(model_path, 'robot.sdf')
    
    # 解析SDF（注意：xacro可以解析SDF，但需要正确配置）
    try:
        doc = xacro.parse(open(sdf_path))
        xacro.process_doc(doc)
        robot_description = doc.toxml()
    except:
        # 如果xacro解析失败，直接读取SDF文件
        with open(sdf_path, 'r') as f:
            robot_description = f.read()
        print("Warning: SDF file loaded directly, not parsed by xacro")
    
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='both',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'robot_description': robot_description
    #     }]
    # )

    # ===== 关节状态发布器（用于可视化） =====
    joint_state_pub_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ===== 启动Ignition Gazebo =====
    world_file = os.path.join(model_path, "world.sdf")
    
    ign_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_ign_gazebo'),
                'launch',
                'ign_gazebo.launch.py'
            )
        ]),
        launch_arguments=[('ign_args', [' -r -v 3 ' + world_file])]
    )

    return LaunchDescription([
        ign_resource_path,
        set_software_render,
        # 启动Gazebo
        ign_gz,
        
        # 生成世界和机器人
        ignition_spawn_world,
        ignition_spawn_entity,
        
        # ROS节点
        # robot_state_publisher,
        joint_state_pub_node,
        bridge_ign2ros2,
        
        # TF发布器
        base_link2base_footprint_tf,
        lidar2base_tf,
        *steering_tfs,  # 展开转向TF列表
        
        # 控制
        # teleop,
        
        # 参数声明
        DeclareLaunchArgument(
            'use_sim_time',
            default_value=use_sim_time,
            description='If true, use simulated clock'
        ),
        DeclareLaunchArgument(
            'world_name',
            default_value=world_name,
            description='World name'
        ),
    ])