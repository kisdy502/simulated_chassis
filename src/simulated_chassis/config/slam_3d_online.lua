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
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- ==================== 前端：仅改雷达范围，其余用源码默认 ====================
-- 源码默认: min_range=1.0, max_range=60.0, voxel_filter_size=0.15
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 24.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.10            -- 恢复源码默认，不用0.05

-- scan matcher 权重：全部用源码默认（translation=5, rotation=4e2），不覆盖
-- 子图大小：用源码默认 160
-- adaptive_voxel_filter：用源码默认

-- ==================== 后端：仅加 fix_z_in_3d，其余用源码默认 ====================
POSE_GRAPH.optimize_every_n_nodes = 90                    -- 源码默认
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3         -- 源码默认
POSE_GRAPH.constraint_builder.min_score = 0.55             -- 源码默认
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6  -- 源码默认

-- 地面机器人：锁死z，防止回环优化拉飞
-- POSE_GRAPH.optimization_problem.fix_z_in_3d = true

return options
