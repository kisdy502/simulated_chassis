include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_footprint",
  odom_frame = "odom",
  -- true: Cartographer发布map→odom和odom→base_footprint，统一TF树
  -- Gazebo中需设置<publish_odom_tf>false</publish_odom_tf>避免冲突
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 0.6,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 累积雷达帧数，用于提高匹配稳定性
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 3
-- 激光范围：与URDF中雷达配置统一为45米
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 25.0
-- 体素滤波：3cm降采样，平衡精度与计算量
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03
-- 缺失数据射线长度：设为雷达最大范围，避免远处误标记为障碍
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0
-- imu
TRAJECTORY_BUILDER_2D.use_imu_data = true
-- 启用实时回环检测，提高前端扫描匹配精度
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动滤波：提高阈值，减少碰撞时异常帧插入
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.3)
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.12
-- 实时相关匹配：增大搜索窗口，碰撞后里程计偏差大也能纠正
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.35
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.)

-- 代价权重（更信激光，纠正碰撞后的里程计漂移）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.05

-- ========== 碰撞鲁棒性：Ceres 扫描匹配 ==========
-- rotation_weight 降低 → 更信任激光对齐，防止子图旋转突变
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 0.8
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 80
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = true

-- ========== 位姿图优化：降里程计权重、增激光约束 ==========
-- 碰撞时里程计不可信，降低其权重（默认约1e5，这里降到1e2~1e3）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e2
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e2
-- 适度降低局部SLAM约束 → 让优化器有能力修复子图间的轻微错位（解决重影）
POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight = 4e4
POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight = 4e4

-- ========== 回环检测：小幅调整，解决重影不引入副作用 ==========
POSE_GRAPH.constraint_builder.min_score = 0.50
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.linear_search_window = 3.0
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.angular_search_window = math.rad(15.)
POSE_GRAPH.optimize_every_n_nodes = 70

return options
