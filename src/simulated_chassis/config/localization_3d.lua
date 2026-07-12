-- localization_3d_optimized.lua
-- Cartographer 3D 纯定位模式（优化版）

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
  lookup_transform_timeout_sec = 0.3,        -- 从0.2增加到0.3，更宽容
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

-- ==================== 3D轨迹构建器（定位优化） ====================

TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 35.0
-- ✅ 从建图的0.05调整为0.10，在速度和精度间取得平衡
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.06
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 180

-- ✅ 定位模式下关闭实时相关扫描匹配（依赖已有地图，节省CPU）
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- ✅ 实时匹配参数（参考建图配置）
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)

-- ✅ 子图帧数适中（定位不需要太多累积）
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 100

-- ✅ Ceres优化权重（参考建图配置）
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 8.0    -- 略低于建图
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 3e2       -- 略低于建图

-- ==================== 后端：纯定位优化 ====================

-- ✅ 优化频率：定位模式可以更慢（减少CPU占用）
POSE_GRAPH.optimize_every_n_nodes = 40      -- 从40增加到60

-- ✅ 约束采样：定位时采样率可以降低
POSE_GRAPH.constraint_builder.sampling_ratio = 0.3

-- ✅ 降低约束阈值，更容易匹配到已有地图（定位宽容度）
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

-- ✅ 全局约束搜索间隔（定位时不需要频繁搜索）
POSE_GRAPH.global_constraint_search_after_n_seconds = 15.0

-- ✅ 优化问题权重（参考建图配置）
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- ✅ 最大活跃节点数限制（控制内存，定位时可以减少）
-- POSE_GRAPH.max_active_nodes = 500

return options