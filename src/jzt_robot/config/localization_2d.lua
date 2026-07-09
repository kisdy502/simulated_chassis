include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  
  -- 核心坐标系
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,  -- 必须启用，用于融合IMU、里程计预测位姿
  use_odometry = true,  -- 启用里程计
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,  -- 仿真用1，真实雷达若运动畸变严重可增大
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.1,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 0.85,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.5,
  landmarks_sampling_ratio = 1.,
}

-- 🔴 重要：必须保持 use_trajectory_builder_2d = true（默认就是 true）
MAP_BUILDER.use_trajectory_builder_2d = true   -- 仍然使用 TrajectoryBuilder，但已设为纯定位

TRAJECTORY_BUILDER.pure_localization_trimmer = {
  max_submaps_to_keep = 3,
}

-- 激光雷达配置
-- 启用在线相关匹配（Real Time Correlative Scan Matcher）
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true

-- 禁用 IMU：Gazebo 仿真中 IMU 数据格式与 Cartographer 不匹配
TRAJECTORY_BUILDER_2D.use_imu_data = true

-- 累积雷达帧数，用于提高匹配稳定性
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 5

-- Ceres 扫描匹配器配置（降低旋转权重防碰撞后定位漂移）
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 5.0

-- Ceres 求解器选项
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 80
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- 实时相关匹配器配置：增大窗口，碰撞后也能找回正确位姿
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.05
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.3
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(2.4)
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.05

-- 降低全局优化频率（每20个节点优化一次），用于纯定位模式以减少计算量
POSE_GRAPH.optimize_every_n_nodes = 20
-- POSE图优化配置：降低里程计权重，碰撞时依赖激光定位
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 1e5
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 1e5
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.55

return options
