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
  publish_frame_projected_to_2d = true,  -- ✅ 发布tf时强制投影到z=0/roll=0/pitch=0，前端local SLAM的z漂移不传导到map→odom
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,                    -- ✅ 关闭2D激光
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,                   -- ✅ 双3D雷达(前+后)
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

-- ✅ 启用3D建图
MAP_BUILDER.use_trajectory_builder_2d = false
MAP_BUILDER.use_trajectory_builder_3d = true
MAP_BUILDER.num_background_threads = 4

-- ✅ 3D 轨迹构建器配置
TRAJECTORY_BUILDER_3D.min_range = 0.5
TRAJECTORY_BUILDER_3D.max_range = 20.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2  -- 双雷达：前+后各1帧累计成完整360°观测后再做扫描匹配
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 180
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.08       -- 8cm 体素滤波

-- 3D 前端开启在线相关扫描匹配(OCSM)兜底：ceres 只做局部精修，初始位姿偏差大时无法恢复
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)

-- 子图帧数 110(比默认 160 小),子图偏小 → 更多回环候选,局部一致性仍足够
TRAJECTORY_BUILDER_3D.submaps.num_range_data = 120

TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0 -- 平移权重
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2    -- 默认 400

POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.65   -- 0.5→0.65,多评估候选回环对
POSE_GRAPH.constraint_builder.min_score = 0.55        -- 0.65→0.60,回收边界回环(3D 默认 0.55)
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2  -- 默认 110
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4      -- 默认 16000（恢复官方默认）
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options