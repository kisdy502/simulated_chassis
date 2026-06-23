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
  num_laser_scans = 0,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 1,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.2,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- ✅ 离线：稍微提高精度
TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 24.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.03      -- 从 0.05 改小，保留更多细节
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 4  -- 从 2 增加到 4，提高匹配稳定性
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 180

-- ✅ 回环检测（与在线一致）
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.12
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)

-- 些尝试性优化，看能不能大幅度提升建图质量
--  每个节点都优化（离线不受实时限制）
POSE_GRAPH.optimize_every_n_nodes = 30
-- ✅ 最终优化：大量迭代打磨结果
POSE_GRAPH.max_num_final_iterations = 300
return options