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

上位机自测（Python）：
  pip install roslibpy
  python3 -c "import roslibpy; c=roslibpy.Ros(host='127.0.0.1', port=9090); c.run()"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ---------------- launch 参数 ----------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    agv_id = LaunchConfiguration("agv_id")
    port = LaunchConfiguration("port")
    address = LaunchConfiguration("address")
    back_up_max_heading_error_deg = LaunchConfiguration("back_up_max_heading_error_deg")

    # 上报白名单：上位机只允许 subscribe 这些 topic（防止整张 ROS 图暴露）
    topics_pub_glob = LaunchConfiguration("topics_pub_glob")
    # 下发白名单：上位机只允许 publish / advertise 这些 topic
    topics_sub_glob = LaunchConfiguration("topics_sub_glob")
    # 服务白名单
    services_glob = LaunchConfiguration("services_glob")

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

        # ==== 上报：上位机 subscribe 的 topic ====
        # 依据 simulated_chassis 的实际话题核对：
        #   /odom              nav_msgs/Odometry        里程计位姿+速度（odom_relay_node 转发）
        #   /tf,/tf_static     tf2_msgs/TFMessage       map->odom->base_link，算位姿必须
        #   /map               nav_msgs/OccupancyGrid   cartographer 占据栅格，1Hz
        #   /plan /local_plan  nav_msgs/Path            全局/局部路径，可视化和监控用
        #   /joint_states      sensor_msgs/JointState   三舵轮转角
        #   /agv/status        AgvStatus                本包的 1Hz 业务状态
        #
        # ⚠️ 刻意不开放的话题：
        #   /points2_1 /points2_2  双 3D 雷达 PointCloud2，单帧 JSON 可达数 MB，走 WS 会打爆链路
        #   /imu                   Gazebo IMU 高频（>100Hz），且已由 Cartographer 融合
        #   /clock                 仿真时钟高频，上位机用消息自带 header.stamp 即可
        #   /scan /amcl_pose       本仿真栈不产生（用 3D 雷达 + Cartographer，非 AMCL）
        DeclareLaunchArgument(
            "topics_pub_glob",
            default_value=(
                "['/odom', '/tf', '/tf_static', '/map', '/plan', '/local_plan', "
                "'/joint_states', '/agv/status']"
            ),
            description="允许上位机订阅（上报）的 topic 白名单",
        ),

        # ==== 下发：上位机 publish 的 topic ====
        # /cmd_vel        —— 对应旧的 velocity_command
        # /initialpose    —— 对应旧的 set_initial_pose
        # /goal_pose      —— 直触发 nav2 行为树（自由导航，占位用）
        DeclareLaunchArgument(
            "topics_sub_glob",
            default_value="['/cmd_vel', '/initialpose', '/goal_pose']",
            description="允许上位机发布（下发）的 topic 白名单",
        ),

        DeclareLaunchArgument(
            "services_glob",
            default_value="['/rosapi/*', '/agv/set_control']",
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
                "params_glob": "['/agv_nav_server/*']",
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
                "params_glob": "['/agv_nav_server/*']",
                "params_timeout": 5.0,
            }
        ],
    )

    return LaunchDescription(args + [agv_nav_server, rosbridge, rosapi])
