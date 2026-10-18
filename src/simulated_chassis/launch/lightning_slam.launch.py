#!/usr/bin/env python3
"""
Lightning-LM 3D 建图启动文件（三舵轮 + 双 3D 雷达仿真）
与 slam.launch.py（Cartographer）平行的对比链路。

用法：
  ros2 launch simulated_chassis lightning_slam.launch.py
  ros2 launch simulated_chassis lightning_slam.launch.py with_ui:=true

流程：
  1. dual_lidar_merge_node：/points2_1 + /points2_2 → /points2_merged（base_link 系）
  2. lightning run_slam_online：LIO 建图（含回环）
  3. 遥控行驶结束后保存地图：
     ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: lm_map}"
     落盘于 <save_dir>/<map_id>/：分块 pcd + global.pcd + map.pgm + map.yaml

注意：
  - 建图过程无 /map、无 TF 发布（与 cartographer 不同），可视化用 Pangolin
    UI（with_ui:=true，需显示器）或结束后用 pcl_viewer 查看 global.pcd。
  - 存图目录默认 $PWD/maps，与 cartographer 地图同目录方便对比。
"""

import os
import tempfile

import yaml as pyyaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("simulated_chassis")
    default_config = os.path.join(pkg_share, "config", "lightning_sim_3d.yaml")

    declared_arguments = [
        DeclareLaunchArgument(
            "config",
            default_value=default_config,
            description="lightning 基础配置 yaml（launch 会注入 save_dir/with_ui 后生成临时配置）",
        ),
        DeclareLaunchArgument(
            "save_dir",
            default_value=os.path.join(os.getcwd(), "maps"),
            description="save_map 服务落盘根目录（最终地图位于 <save_dir>/<map_id>/）",
        ),
        DeclareLaunchArgument(
            "with_ui",
            default_value="false",
            description="是否启用 Pangolin 3D UI（需要显示器；docker 无头环境保持 false）",
        ),
        DeclareLaunchArgument(
            "use_sim_time", default_value="true", description="使用仿真时间"
        ),
    ]

    def launch_setup(context):
        # 生成注入运行时参数的临时配置文件
        base_cfg = LaunchConfiguration("config").perform(context)
        save_dir = LaunchConfiguration("save_dir").perform(context)
        with_ui = LaunchConfiguration("with_ui").perform(context).lower() in ("true", "1")
        # launch 字典参数保持 Python 类型：bool 参数必须传 bool，传字符串会让节点启动即崩
        use_sim_time_bool = LaunchConfiguration("use_sim_time").perform(context).lower() in ("true", "1")
        use_sim_time = LaunchConfiguration("use_sim_time").perform(context)

        with open(base_cfg, "r", encoding="utf-8") as f:
            cfg = pyyaml.safe_load(f)
        cfg.setdefault("system", {})["save_dir"] = os.path.abspath(save_dir)
        cfg["system"]["with_ui"] = with_ui

        cfg_file = os.path.join(tempfile.gettempdir(), f"lightning_cfg_slam_{os.getpid()}.yaml")
        with open(cfg_file, "w", encoding="utf-8") as f:
            pyyaml.safe_dump(cfg, f, allow_unicode=True)

        return [
            # 1. 双雷达合流（先于 lightning 启动）
            Node(
                package="simulated_chassis",
                executable="dual_lidar_merge_node",
                name="dual_lidar_merge_node",
                output="screen",
                 parameters=[
                     {
                         "target_frame": "base_link",
                         "time_mode": "zero",
                         "scan_period_ms": 66.7,
                         "use_sim_time": use_sim_time_bool,
                     }
                 ],
            ),
            # 2. lightning 在线建图（gflags 与 rclcpp 参数共存需 "--" 分隔）
            #    [补丁] SLAM 亦发布 /tracked_pose（LIO 逐帧位姿），由同款
            #    tracked_pose_tf_node 重锚定为 map→odom TF —— 建图期间
            #    RViz/上位机即可看实时位姿与 /map 栅格（与 Cartographer 体验对齐）
            TimerAction(
                period=1.5,
                actions=[
                    Node(
                        package="lightning",
                        executable="run_slam_online",
                        name="lightning_slam",
                        output="screen",
                        arguments=[
                            "--config", cfg_file,
                            "--",
                            "--ros-args",
                            "-p", "use_sim_time:=" + use_sim_time,
                        ],
                    ),
                    Node(
                        package="simulated_chassis",
                        executable="tracked_pose_tf_node",
                        name="tracked_pose_tf_node",
                        output="screen",
                        parameters=[{
                            "use_sim_time": use_sim_time_bool,
                            "map_frame": "map",
                            "odom_frame": "odom",
                            "tracking_frame": "base_link",
                            "tracked_pose_topic": "/tracked_pose",
                        }],
                    ),
                ],
            ),
        ]

    return LaunchDescription(
        [
            LogInfo(msg=["Lightning-LM 3D 建图模式（dev-lightning 对比分支）"]),
            LogInfo(msg=['保存地图：ros2 service call /lightning/save_map lightning/srv/SaveMap "{map_id: lm_map}"']),
            *declared_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )
