-- slam_3d_offline.lua
-- Cartographer 3D 离线精细建图配置

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

-- ==================== 离线模式：最大化精度 ====================

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
-- ✅ 离线模式可以用更多线程
MAP_BUILDER.num_background_threads = 8

-- ==================== 3D 轨迹构建器（超精细）====================
TRAJECTORY_BUILDER_3D.min_range = 0.15
TRAJECTORY_BUILDER_3D.max_range = 25.0

-- ✅ 更小的体素 = 更精细的地图
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.01

TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- ✅ 超高分辨率子图
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.02   -- 2cm 分辨率！
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.08

TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.02
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 250
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 8.0

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 0.08
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 80
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 25.0

-- ✅ 更小的子图，减少局部漂移
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 40

-- 运动滤波
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.3
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.05
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(1.0)

-- 禁用实时相关扫描匹配
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- IMU 外推器
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 3.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 0.5

-- Ceres 扫描匹配器
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 15.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 8.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 50
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 2

-- ==================== 全局优化（离线可以更激进）====================
-- ✅ 每个节点都优化（离线不受实时限制）
POSE_GRAPH.optimize_every_n_nodes = 1

POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5

POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_xy_search_window = 8.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.linear_z_search_window = 2.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.angular_search_window = math.rad(20.0)
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth = 10
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth = 4

POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_0 = 15.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.occupied_space_weight_1 = 8.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.translation_weight = 1.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.rotation_weight = 1.0
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.max_num_iterations = 100
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d.ceres_solver_options.num_threads = 2

-- ✅ 优化问题：离线可以用更多迭代
POSE_GRAPH.optimization_problem.huber_scale = 5e2
POSE_GRAPH.optimization_problem.acceleration_weight = 1e-2
POSE_GRAPH.optimization_problem.rotation_weight = 1e-2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e3
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e4
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3
POSE_GRAPH.optimization_problem.log_solver_summary = true
POSE_GRAPH.optimization_problem.ceres_solver_options.use_nonmonotonic_steps = false
POSE_GRAPH.optimization_problem.ceres_solver_options.max_num_iterations = 200
POSE_GRAPH.optimization_problem.ceres_solver_options.num_threads = 8

-- ✅ 最终优化：大量迭代打磨结果
POSE_GRAPH.max_num_final_iterations = 1000

POSE_GRAPH.constraint_builder.log_matches = true
POSE_GRAPH.log_residual_histograms = true

return options