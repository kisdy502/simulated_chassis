-- localization_3d.lua
-- Cartographer 3D 纯定位（基于已验证的在线建图参数对齐）
-- 原则：建图能稳定匹配的参数，定位直接复用，只改 CPU 占用相关的节奏

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
  publish_frame_projected_to_2d = true,                  -- 定位投影到2D，给Nav2用
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.3,                    -- 定位稍宽容（仿真TF可能延迟）
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- ==== 3D轨迹构建器：与建图完全一致，保证点云特征空间吻合 ====
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 30.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.08           -- 对齐建图（定位不需要更细）
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 180

TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.2
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(3.0)

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100       -- 定位用更小子图，及时滑动窗口抛弃旧数据

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0   -- 对齐建图
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2       -- 对齐建图

-- ==== 后端：纯定位（仅放宽节奏，不动匹配门槛） ====
POSE_GRAPH.optimize_every_n_nodes = 60                   -- 定位下不需要频繁优化（建图是40）
POSE_GRAPH.constraint_builder.sampling_ratio = 0.4       -- 定位降低采样率，省CPU
POSE_GRAPH.constraint_builder.min_score = 0.65           -- 对齐建图（地图好，反而要更高门槛防误匹配）
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0  -- 定位下不需要频繁全局搜索

POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- ✅ 锁定 Z 轴（与建图一致，地面机器人防止 Z 漂移）
POSE_GRAPH.optimization_problem.fix_z_in_3d = true

return options
