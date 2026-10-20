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
  # simulated_chassis 默认同时托管定位和 Nav2，不再手动启动 navigation.launch.py：
  # 建图时停 Nav2，定位 TF 就绪后重新启动；速度转发常驻，建图仍可遥控。
  # 旧流程传 manage_navigation:=false，手动 navigation launch 传 include_localization:=false。
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py pbstream_file:=$PWD/maps/my_map/my_map.pbstream
  # 换机器人（定位/建图 launch 从该包拉起）：
  ros2 launch agv_bridge_v2 agv_rosbridge.launch.py robot_package:=zioneer_robot

上位机自测（Python）：
  pip install roslibpy
  python3 -c "import roslibpy; c=roslibpy.Ros(host='127.0.0.1', port=9090); c.run()"
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, EmitEvent
from ament_index_python.packages import get_package_prefix
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _workspace_root():
    """定位 colcon 工作区根目录(地图等运行时产物的固定锚点)。

    优先级:环境变量 SIM_WS_ROOT(显式覆盖) > 从本 launch 文件真实路径向上
    找同时含 src/ 与 install/ 的目录(symlink-install 时解析到 src 包路径,
    拷贝安装时停在 install 一样能命中)。找不到返回 None。"""
    env = os.environ.get("SIM_WS_ROOT")
    if env:
        return env
    try:
        here = os.path.dirname(os.path.realpath(__file__))
    except Exception:
        return None
    for _ in range(6):
        here = os.path.dirname(here)
        if here in ("", "/"):
            break
        if os.path.isdir(os.path.join(here, "src")) and os.path.isdir(os.path.join(here, "install")):
            return here
    return None


def _resolve_maps_dir(context):
    """地图根目录解析(决定 save_map 落盘与 list_maps 扫描位置)。

    优先级:显式 maps_dir 参数 > 从 pbstream_file 推导 > 工作区根 maps/
    （固定路径,不随 bridge 启动 cwd 漂移;历史教训:曾解析成启动目录本身,
    地图散落到 ~/workspace/<name>)。每次建图在 <maps_dir>/<地图名>/ 下
    生成独立子目录。"""
    root = _workspace_root()
    ws_maps = os.path.join(root, "maps") if root else None

    def _clamp(path):
        """防漂移：解析结果必须位于工作区 maps/ 之下。

        历史教训：pbstream_file 传 ../maps 或目录路径时，两级 dirname 推导
        会落到工作区上一级（地图散落成 D:/github/map1019）。自定义场景用
        SIM_WS_ROOT 环境变量改锚点，而不是传工作区外的相对路径。"""
        if not ws_maps:
            return path
        ap = os.path.abspath(path)
        if ap != ws_maps and not ap.startswith(ws_maps + os.sep):
            print(f"[agv_rosbridge] maps_dir '{ap}' 不在工作区 maps/ 下，已纠正为 '{ws_maps}'"
                  f"（自定义位置请设置 SIM_WS_ROOT 环境变量）")
            return ws_maps
        return ap

    explicit = LaunchConfiguration("maps_dir").perform(context).strip()
    if explicit and explicit != "maps":
        return _clamp(explicit)
    pb = os.path.abspath(LaunchConfiguration("pbstream_file").perform(context).strip())
    if pb:
        if os.path.isdir(pb):
            return _clamp(os.path.dirname(pb) or "maps")
        derived = os.path.dirname(os.path.dirname(pb))
        if derived:
            return _clamp(derived)
    return ws_maps if ws_maps else "maps"


def _map_3d_server(context):
    """三维地图后台导出 + HTTP 文件服务（8089）。

    同时支持 Cartographer pbstream 与 lightning global.pcd 两种地图来源，
    按 slam_algorithm 选择来源，避免同名旧地图混用；
    上位机统一通过 /maps/<name>/status 与 /cloud.ply 消费。"""
    if LaunchConfiguration("enable_map_3d").perform(context).lower() != "true":
        return []
    try:
        executable_dir = os.path.join(get_package_prefix("agv_map_3d"), "lib", "agv_map_3d")
    except Exception:
        print("[agv_rosbridge] agv_map_3d 未安装，跳过三维地图服务")
        return []
    for executable in ("map_3d_server", "pbstream_to_ply", "pcd_to_ply"):
        if not os.access(os.path.join(executable_dir, executable), os.X_OK):
            print("[agv_rosbridge] 三维地图安装不完整，请重新编译 agv_map_3d，跳过三维地图服务")
            return []
    return [ExecuteProcess(
        cmd=[os.path.join(executable_dir, "map_3d_server"),
             "--exporter", os.path.join(executable_dir, "pbstream_to_ply"),
             "--exporter-pcd", os.path.join(executable_dir, "pcd_to_ply"),
             "--source-format", "pcd" if LaunchConfiguration("slam_algorithm").perform(context) == "lightning" else "pbstream",
             "--maps-dir", _resolve_maps_dir(context),
             "--address", "0.0.0.0",
             "--port", LaunchConfiguration("map_3d_port")],
        output="screen",
    )]


def _agv_nav_server_node(context):
    use_sim_time = LaunchConfiguration("use_sim_time")
    agv_id = LaunchConfiguration("agv_id")
    back_up_max_heading_error_deg = LaunchConfiguration("back_up_max_heading_error_deg")
    pbstream_file = LaunchConfiguration("pbstream_file")
    robot_package = LaunchConfiguration("robot_package")
    localization_launch_file = LaunchConfiguration("localization_launch_file")
    slam_launch_file = LaunchConfiguration("slam_launch_file")
    slam_algorithm = LaunchConfiguration("slam_algorithm")
    lightning_slam_launch_file = LaunchConfiguration("lightning_slam_launch_file")
    lightning_localization_launch_file = LaunchConfiguration("lightning_localization_launch_file")
    manage_arg = LaunchConfiguration("manage_navigation").perform(context).lower()
    manage_navigation = (robot_package.perform(context) == "simulated_chassis"
                         if manage_arg == "auto" else manage_arg == "true")
    if manage_navigation and robot_package.perform(context) != "simulated_chassis":
        raise RuntimeError("manage_navigation currently supports simulated_chassis only")
    nodes = [Node(
        package="agv_bridge_v2",
        executable="agv_nav_server",
        name="agv_nav_server",
        output="screen",
        emulate_tty=True,
        sigterm_timeout='30.0',
        sigkill_timeout='5.0',
        on_exit=[EmitEvent(event=Shutdown(reason='AGV navigation server exited'))],
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
                "maps_dir": _resolve_maps_dir(context),
                "pbstream_file": pbstream_file,
                "robot_package": robot_package,
                "manage_navigation": manage_navigation,
                'navigation_use_rviz': ParameterValue(LaunchConfiguration('navigation_use_rviz'), value_type=bool),
                "localization_launch_file": localization_launch_file,
                "slam_launch_file": slam_launch_file,
                "slam_algorithm": slam_algorithm,
                "lightning_slam_launch_file": lightning_slam_launch_file,
                "lightning_localization_launch_file": lightning_localization_launch_file,
            }
        ],
    )]
    if manage_navigation:
        nodes.append(Node(
            package="simulated_chassis", executable="cmd_vel_relay_node",
            name="bridge_cmd_vel_relay_node", output="screen",
            parameters=[{"input_topic": "/cmd_vel",
                         "output_topic": "/three_wheel_base_controller/cmd_vel",
                         "use_sim_time": use_sim_time}],
        ))
    return nodes


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
        DeclareLaunchArgument('navigation_use_rviz',
            default_value=EnvironmentVariable('START_RVIZ', default_value='true'),
            description='托管导航是否打开 RViz；组合机械臂 profile 固定关闭'),
        DeclareLaunchArgument("manage_navigation", default_value="auto",
                              description="auto: simulated_chassis 托管 Nav2；false: 手动启动导航"),
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
        # save_map 保存后自动重启定位）。simulated_chassis 默认托管导航，
        # 不要另启 navigation launch；其他包或 manage_navigation:=false 时，
        # 手动 navigation launch 需传 include_localization:=false。
        DeclareLaunchArgument(
            "maps_dir",
            default_value="maps",
            description="地图根目录（每张地图位于 <maps_dir>/<map_name>/）；缺省自动取 pbstream_file 上两级目录，不随启动 cwd 变化",
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
        DeclareLaunchArgument(
            "slam_algorithm",
            default_value="cartographer",
            choices=["cartographer", "lightning"],
            description="SLAM 后端：cartographer（pbstream 三件套）或 lightning（点云地图目录）",
        ),
        DeclareLaunchArgument(
            "lightning_slam_launch_file",
            default_value="lightning_slam.launch.py",
            description="lightning 建图 launch 文件名（在 robot_package 内）",
        ),
        DeclareLaunchArgument(
            "lightning_localization_launch_file",
            default_value="lightning_loc.launch.py",
            description="lightning 定位 launch 文件名（在 robot_package 内）",
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
                "'/agv/list_maps', '/agv/start_mapping', '/agv/save_map', '/agv/relocalize', "
                "'/agv/arm_move']"
            ),
            description="允许上位机调用的 service 白名单",
        ),
    ]

    # ---------------- 1. AGV 瘦节点 ----------------
    agv_nav_server = OpaqueFunction(function=_agv_nav_server_node)

    # ---------------- 2. rosbridge_websocket ----------------
    rosbridge = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        # 日志级别 WARN：默认 INFO 会为每条分片消息打一行
        # （网页订阅 /map 时 1Hz × 4 分片 = 每秒刷屏），只留告警和错误。
        arguments=["--ros-args", "--log-level", "WARN"],
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
                "params_glob": ParameterValue("['robot_description', '/agv_nav_server/*', '/robot_state_publisher/*']", value_type=str),
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
                "params_glob": ParameterValue("['robot_description', '/agv_nav_server/*', '/robot_state_publisher/*']", value_type=str),
                "params_timeout": 5.0,
            }
        ],
    )

    # 常驻地图保存节点：提前建立 /map 订阅，避免 map_saver_cli
    # 每次冷启动时 DDS 发现占用 Humble 默认 2s 等待窗口。
    map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            # 10s→30s：CPU 饱和时 occupancy_grid 的 /map 迟滞，10s 窗口收不到图
            # 就报 result=false（save_map 失败回退的根因之一）
            "save_map_timeout": 30.0,
            "free_thresh_default": 0.25,
            "occupied_thresh_default": 0.65,
        }],
    )

    map_saver_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_saver",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_saver"],
        }],
    )

    args += [
        DeclareLaunchArgument("enable_map_3d", default_value="true",
                              description="启动三维地图后台导出 + HTTP 文件服务（pbstream/lightning 双格式）"),
        DeclareLaunchArgument("map_3d_port", default_value="8089",
                              description="三维地图 HTTP 服务端口"),
    ]

    return LaunchDescription(args + [
        agv_nav_server,
        map_saver,
        map_saver_lifecycle_manager,
        rosbridge,
        rosapi,
        OpaqueFunction(function=_map_3d_server),
    ])
