#!/usr/bin/env python3
"""
agv_rosbridge.launch.py —— rosbridge 架构下的 AGV 对接 launch

启动内容：
  1. agv_nav_server     瘦节点：只做路径生成 + nav2 调度，暴露 /agv/* 接口
  2. rosbridge_websocket JSON over WebSocket 服务，上位机从这里接入
  3. rosapi              图查询 / 参数服务（上位机用它下载消息定义）

替代关系：
  旧 agv_bridge_node 的 WebSocketServer + HttpApiClient + bean/* 全部由
  rosbridge_server 承担；本 launch 不需要再编译 libwebsockets / libcurl 业务代码。

用法：
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py port:=9090 agv_id:=three_wheel_agv
  # 托管定位（启用 /agv/load_map 切图，navigation launch 需 include_localization:=false）：
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py pbstream_file:=$PWD/maps/my_map.pbstream
  # 换机器人（定位/建图 launch 从该包拉起）：
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py robot_package:=zioneer_robot

上位机自测（Python）：
  pip install roslibpy
  python3 -c "import roslibpy; c=roslibpy.Ros(host='127.0.0.1', port=9090); c.run()"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    # ---------------- launch 参数 ----------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    agv_id = LaunchConfiguration("agv_id")
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    back_up_max_heading_error_deg = LaunchConfiguration("back_up_max_heading_error_deg")
    maps_dir = LaunchConfiguration("maps_dir")
    pbstream_file = LaunchConfiguration("pbstream_file")
    robot_package = LaunchConfiguration("robot_package")
    localization_launch_file = LaunchConfiguration("localization_launch_file")
    slam_launch_file = LaunchConfiguration("slam_launch_file")

    # ⚠️ rosbridge 语义（以 humble 分支源码为准）：
    #   topics_sub_glob  = 上位机「能订阅」（收数据）的白名单
    #   topics_pub_glob  = 上位机「能发布」（下数据）的白名单
    # 且四个 glob 参数在 rosbridge 侧声明为 STRING 类型（内容是 "['a','b']" 形式的字面量），
    # launch_ros 会把形如列表的字符串自动解析成数组，必须用 ParameterValue(value_type=str) 钉死为字符串。
    topics_pub_glob = ParameterValue(LaunchConfiguration("topics_pub_glob"), value_type=str)
    topics_sub_glob = ParameterValue(LaunchConfiguration("topics_sub_glob"), value_type=str)
    services_glob = ParameterValue(LaunchConfiguration("services_glob"), value_type=str)

    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="是否使用仿真时间（Gazebo /clock）"),
        DeclareLaunchArgument("agv_id", default_value="AGV001",
                              description="AGV 唯一标识，用于 TF 的 child_frame_id"),
        DeclareLaunchArgument("port", default_value="9090",
                              description="rosbridge WebSocket 端口"),
        DeclareLaunchArgument("address", default_value="0.0.0.0",
                              description="rosbridge 监听地址"),

        # 倒车朝向偏差阈值：倒车绝不自动旋转，偏差超过此值直接 FAILED，
        # 由调度系统决定先摆正还是改用前进指令。设很大 = 不做检查。
        DeclareLaunchArgument(
            "back_up_max_heading_error_deg",
            default_value="20.0",
            description="倒车时车尾对目标的朝向偏差容忍上限（度）",
        ),

        # ==== 地图管理（上位机 /agv/get_map、/agv/load_map、/agv/list_maps、
        #      /agv/start_mapping、/agv/save_map）====
        # 机器人包标准契约：包内需提供 localization.launch.py 与 slam.launch.py。
        # pbstream_file 非空时，定位由 agv_nav_server 以子进程托管（load_map 切图、
        # save_map 保存后自动重启定位）；此时 navigation launch 需传
        # include_localization:=false，避免 cartographer 双开。
        DeclareLaunchArgument(
            "maps_dir",
            default_value="maps",
            description="地图目录（pgm/yaml/pbstream 三件套所在，相对启动 cwd）",
        ),
        DeclareLaunchArgument(
            "pbstream_file",
            default_value="",
            description="初始定位地图 pbstream（空 = 启动后无定位，可先 start_mapping 建图）",
        ),
        DeclareLaunchArgument(
            "robot_package",
            default_value="jzt_robot",
            description="机器人包名（定位/建图 launch 所在包，如 simulated_chassis / zioneer_robot）",
        ),
        DeclareLaunchArgument(
            "localization_launch_file",
            default_value="localization.launch.py",
            description="定位 launch 文件名（在 robot_package 内）",
        ),
        DeclareLaunchArgument(
            "slam_launch_file",
            default_value="slam.launch.py",
            description="建图 launch 文件名（在 robot_package 内）",
        ),

        # ==== 下发：上位机 publish 的 topic（rosbridge 语义：gates advertise/publish）====
        # /cmd_vel        —— 手动点动（经 cmd_vel_relay 转发到底盘）
        # /initialpose    —— 重定位（Cartographer）
        # /goal_pose      —— 自由导航到任意目标点（nav2 行为树直接接单）
        DeclareLaunchArgument(
            "topics_pub_glob",
            default_value="['/cmd_vel', '/initialpose', '/goal_pose']",
            description="允许上位机发布（下发）的 topic 白名单",
        ),

        # ==== 上报：上位机 subscribe 的 topic（rosbridge 语义：gates subscribe）====
        # 依据 simulated_chassis 的实际话题核对：
        #   /odom              nav_msgs/Odometry        里程计位姿+速度（odom_relay_node 转发）
        #   /tf,/tf_static     tf2_msgs/TFMessage       map->odom->base_link，算位姿必须
        #   /map               nav_msgs/OccupancyGrid   cartographer 占据栅格，1Hz
        #   /plan /local_plan  nav_msgs/Path            全局/局部路径，可视化和监控用
        #   /joint_states      sensor_msgs/JointState   三舵轮转角
        #   /agv/status        AgvStatus                本包的 1Hz 业务状态
        # 加 pointcloud_to_laserscan 后可补 '/scan'
        DeclareLaunchArgument(
            "topics_sub_glob",
            default_value=(
                "['/odom', '/tf', '/tf_static', '/map', '/plan', '/local_plan', "
                "'/joint_states', '/agv/status', '/scan_1', '/scan_2', '/agv/pose']"
            ),
            description="允许上位机订阅（上报）的 topic 白名单",
        ),

        DeclareLaunchArgument(
            "services_glob",
            default_value=(
                "['/rosapi/*', '/agv/set_control', '/agv/get_map', '/agv/load_map', "
                "'/agv/list_maps', '/agv/start_mapping', '/agv/save_map']"
            ),
            description="允许上位机调用的 service 白名单",
        ),
    ]

    # ---------------- 1. AGV 瘦节点 ----------------
    agv_nav_server = Node(
        package="agv_bridge_v2",
        executable="agv_nav_server",
        name="agv_nav_server",
        output="screen",
        emulate_tty=True,
        # 必须运行在根命名空间：NavigationManager 内部用相对名创建
        # follow_path / spin / navigate_to_pose 客户端。
        namespace="/",
        parameters=[
            {
                "agv_id": agv_id,
                "use_sim_time": use_sim_time,
                "feedback_interval_ms": 400,
                "battery_level": 100.0,
                "enable_tf_broadcast": True,
                "back_up_max_heading_error_deg": back_up_max_heading_error_deg,
                "maps_dir": maps_dir,
                "pbstream_file": pbstream_file,
                "robot_package": robot_package,
                "localization_launch_file": localization_launch_file,
                "slam_launch_file": slam_launch_file,
            }
        ],
    )

    # ---------------- 2. rosbridge_websocket ----------------
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "port": port,
                "address": address,
                "url_path": "/",

                # 心跳：取代旧 bridge 里手写的 {"type":"heartbeat"}。
                # 服务端每 5s 发 WS ping，15s 无 pong 即认为链路断开。
                "websocket_ping_interval": 5.0,
                "websocket_ping_timeout": 15.0,

                # 客户端断开后延迟 10s 才回收订阅（容忍上位机短暂重连）
                "unregister_timeout": 10.0,
                # 大消息分片之后保留 600s
                "fragment_timeout": 600,
                # 单条消息上限 10MB —— Path 很长时才需要考虑调大
                "max_message_size": 10000000,
                # 多客户端同时订阅高频 topic 时保持默认 0，避免整体降速
                "delay_between_messages": 0.0,
                # permessage-deflate，能显著压缩 /map、/plan 这类文本 JSON
                "use_compression": True,
                # action 目标与 service 调用放到独立线程，避免阻塞 WS 事件循环
                "send_action_goals_in_new_thread": True,
                "call_services_in_new_thread": True,
                "default_call_service_timeout": 5.0,

                # ==== 访问控制（务必配置，否则等于把整张 ROS 图敞开）====
                "topics_pub_glob": topics_pub_glob,
                "topics_sub_glob": topics_sub_glob,
                "services_glob": services_glob,
                # 参数只允许访问本节点，防止远端改掉 nav2 关键参数
                "params_glob": ParameterValue("['/agv_nav_server/*']", value_type=str),
            }
        ],
    )

    # ---------------- 3. rosapi ----------------
    # 上位机用它列出 topic / 下载消息定义（自动生成 DTO 的依据）
    rosapi = Node(
        package="rosapi",
        executable="rosapi_node",
        name="rosapi",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "topics_pub_glob": topics_pub_glob,
                "topics_sub_glob": topics_sub_glob,
                "services_glob": services_glob,
                "params_glob": ParameterValue("['/agv_nav_server/*']", value_type=str),
                "params_timeout": 5.0,
            }
        ],
    )

    return LaunchDescription(args + [agv_nav_server, rosbridge, rosapi])
