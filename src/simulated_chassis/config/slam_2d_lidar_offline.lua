-- Cartographer 2D 离线建图（2D 雷达版机器人 three_wheel_chassis_2d.xacro）
--
-- 传感器输入与 slam_2d_lidar_online.lua 完全一致（/scan_1 + /scan_2 + /odom，
-- bag 录制话题见 README），离线专属差异参考 slam_3d_offline.lua：
--   更多后台线程、更密的子图/优化节奏、更大的最终迭代数，用满算力打磨地图。
--
-- 用法：
--   ros2 launch simulated_chassis slam_offline.launch.py \
--       configuration_basename:=slam_2d_lidar_offline.lua \
--       bag_filenames:=<bag> save_state_filename:=<pbstream>

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
  num_laser_scans = 2,                    -- 双雷达：左前 /scan_1，右后 /scan_2
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
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

-- ==== 2D 建图核心（与在线 slam_2d_lidar_online.lua 对齐，保证特征一致） ====
MAP_BUILDER.use_trajectory_builder_2d = true

TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 20.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2  -- 双雷达成对累积（前后各1帧）
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
-- 离线不受实时限制，子图更密、匹配窗口可略放宽
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

-- ==== 全局优化（沿用在线验证过的稳定权重） ====
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.5
POSE_GRAPH.constraint_builder.min_score = 0.65
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

-- ==== 离线专属：用满算力打磨最终地图 ====
MAP_BUILDER.num_background_threads = 8                   -- 离线可用更多线程
POSE_GRAPH.max_num_final_iterations = 300                -- 默认200（1000 会闪退，300 已足够）

return options
