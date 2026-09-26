-- Cartographer 2D 在线建图（双雷达水平扫描版）
--
-- 思路：平面地板 AGV 的建图/导航不需要 3D SLAM。Gazebo gpu_lidar 自带的
-- 2D LaserScan 输出（/scan_1、/scan_2）取垂直视场中心波束——两雷达各前倾
-- 5°、垂直中心 +5°，恰好合成水平波束（离地 0.25m，高于底盘顶 0.24m）：
--   - 波束平行于地面 → 地面回波在几何上不存在 → 2D 地图无灰点
--   - 波束越过自家底盘 → 四角自打点不存在（0.72m 处对方雷达穹顶由 min_range 挡掉）
--   - 2D SLAM 没有 z 轴 → z 漂移在数学上不存在
-- 3D 点云仍由 nav2 代价地图直接消费（其 min/max_obstacle_height 自带高度带）。
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,                    -- 双雷达：前 /scan_1，后 /scan_2
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,                   -- 不再消费 3D 点云
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 2D 轨迹构建器：min_range 挡掉对方雷达穹顶(0.72m)等近场自打点
TRAJECTORY_BUILDER_2D.min_range = 0.8
TRAJECTORY_BUILDER_2D.max_range = 25.0
TRAJECTORY_BUILDER_2D.use_imu_data = false         -- 轮式里程计的航向已足够
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.65
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
