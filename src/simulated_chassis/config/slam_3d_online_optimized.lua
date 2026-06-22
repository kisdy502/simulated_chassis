-- slam_3d_online_optimized.lua
-- Cartographer 3D 优化配置（在线模式）

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
  imu_sampling_ratio = 1.,          -- ✅ 提高 IMU 采样率
  landmarks_sampling_ratio = 1.,
}

-- ==================== 地图构建器 ====================
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 6   -- ✅ 增加后台线程（根据CPU调整）

-- ==================== 3D 轨迹构建器（精细建图）====================
TRAJECTORY_BUILDER_3D.min_range = 0.15
TRAJECTORY_BUILDER_3D.max_range = 25.0

-- ✅ 体素滤波：更小 = 更精细（但计算量更大）
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.1

-- ✅ 累积帧数：10Hz雷达，累积1帧 = 100ms，平衡延迟与密度
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1

-- ✅ 子图分辨率：核心参数！越小越精细
TRAJECTORY_BUILDER_3D.submaps.high_resolution = 0.03   -- 高分辨率网格（近距离）
TRAJECTORY_BUILDER_3D.submaps.low_resolution = 0.10    -- 低分辨率网格（远距离）

-- ✅ 自适应体素滤波：控制点云密度
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_length = 0.03
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.min_num_points = 200
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range = 10.0

TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_length = 0.10
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.min_num_points = 100
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range = 25.0

-- ✅ 子图大小：更小 = 更少的局部漂移
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 60

-- ✅ 运动滤波：避免插入过多相似帧
TRAJECTORY_BUILDER_3D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_3D.motion_filter.max_distance_meters = 0.1
TRAJECTORY_BUILDER_3D.motion_filter.max_angle_radians = math.rad(2.0)

-- ✅ 禁用实时相关扫描匹配，使用 IMU 外推器
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- ✅ IMU 外推器配置（3D 建图关键）
TRAJECTORY_BUILDER_3D.pose_extrapolator.use_imu_based = true
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_acceleration_weight = 3.0
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.imu_rotation_weight = 0.5
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.ceres_solver_options.max_num_iterations = 10
TRAJECTORY_BUILDER_3D.pose_extrapolator.imu_based.ceres_solver_options.num_threads = 1

-- ✅ Ceres 扫描匹配器权重
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0 = 10.0   -- 高分辨率权重
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1 = 5.0    -- 低分辨率权重
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 1.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps = false
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.max_num_iterations = 30
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.ceres_solver_options.num_threads = 1

-- ==================== 回环检测与全局优化 ====================
POSE_GRAPH.optimize_every_n_nodes = 60   -- 与 num_range_data 相同，每完成一个子图优化一次

-- ✅ 约束构建器
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.60
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3


return options