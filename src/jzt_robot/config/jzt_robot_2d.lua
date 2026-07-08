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
  num_laser_scans = 1,
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
  imu_sampling_ratio = 0.2,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 累积雷达帧数，用于提高匹配稳定性
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
-- 激光范围：与URDF中雷达配置统一为45米
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 25.0
-- 体素滤波：3cm降采样，平衡精度与计算量
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.03
-- 缺失数据射线长度：设为雷达最大范围，避免远处误标记为障碍
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 25.0
-- 禁用IMU：Gazebo仿真中IMU数据格式与Cartographer不匹配
TRAJECTORY_BUILDER_2D.use_imu_data = false
-- 启用实时回环检测，提高前端扫描匹配精度
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 运动滤波：0.2°角度阈值，对小角度变化更敏感
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(0.2)
-- 实时相关匹配
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.12
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(1.)

-- 代价权重（更信激光）
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.translation_delta_cost_weight = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.rotation_delta_cost_weight = 0.1

POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.7

return options
