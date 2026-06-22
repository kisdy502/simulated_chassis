-- localization_3d.lua
-- Cartographer 3D 纯定位模式

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
  imu_sampling_ratio = 1.,        -- 定位建议用全量 IMU
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- ==================== 纯定位模式关键配置 ====================

-- 前端参数（可以比建图更宽松，因为地图已存在）
TRAJECTORY_BUILDER_3D.min_range = 0.2
TRAJECTORY_BUILDER_3D.max_range = 24.0
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.15   -- 定位可以用更粗体素，加快速度
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 180

-- 定位模式下建议关闭实时相关扫描匹配（依赖已有地图）
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = false

-- ==================== 后端：纯定位设置 ====================

-- 降低约束阈值，更容易匹配到已有地图
POSE_GRAPH.constraint_builder.min_score = 0.45
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.50

-- ✅ 关键：降低采样率，加快约束搜索（地图已知，不需要全局搜索）
POSE_GRAPH.global_sampling_ratio = 0.001

-- ✅ 关键：增加全局约束搜索间隔（定位时不需要频繁搜索）
POSE_GRAPH.global_constraint_search_after_n_seconds = 30.

return options