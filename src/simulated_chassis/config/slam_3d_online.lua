include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  -- 分工模式：控制器发 odom->base_footprint TF(100Hz)+/odom，
  -- cartographer 只发 map->odom(50Hz)。
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,  -- 发布tf时强制投影到z=0/roll=0/pitch=0，前端local SLAM的z漂移不传导到map→odom
  -- ✅ TF 发布方案（配套 tracked_pose_tf_node，解决两种抖动）：
  -- 1) 外推开+carto直发map→odom：值=外推到now的SLAM位姿∘t_slam旧时刻odom位姿⁻¹，锚定时刻不一致，
  --    与控制器 odom→base 组合会把 t_slam→now 的运动计两次 → 前进-回退锯齿抖动；
  -- 2) 外推关：map→odom 只随 local SLAM 结果(约10Hz+计算延迟)步进 → RViz 机器人卡顿。
  -- 方案：外推只用于 /tracked_pose（高频平滑、z已投影），carto 不发 TF（publish_to_tf=false），
  --       由 tracked_pose_tf_node 在同一时间戳查 odom→base 重锚定发布 map→odom，平滑且自洽。
  use_pose_extrapolator = true,
  publish_to_tf = false,
  publish_tracked_pose = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 0,                    -- ✅ 关闭2D激光
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 2,                   -- ✅ 双3D雷达(前+后)
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 20e-3,
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
TRAJECTORY_BUILDER_3D.min_range = 0.55
TRAJECTORY_BUILDER_3D.max_range = 35.0
TRAJECTORY_BUILDER_3D.num_accumulated_range_data = 2  -- ✅ 双雷达：每个雷达1帧，累计2帧后做一次扫描匹配
TRAJECTORY_BUILDER_3D.rotational_histogram_size = 120
TRAJECTORY_BUILDER_3D.voxel_filter_size = 0.10       -- 10cm 体素滤波

-- 3D 前端开启在线相关扫描匹配(OCSM)：库房重复结构下 Ceres 孤军匹配会滑向错误平行墙
TRAJECTORY_BUILDER_3D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.linear_search_window = 0.15
TRAJECTORY_BUILDER_3D.real_time_correlative_scan_matcher.angular_search_window = math.rad(5.0)

TRAJECTORY_BUILDER_3D.submaps.num_range_data = 120
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability = 0.55
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10.0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 4e2

-- 小步快跑：优化频率高、每次落地的约束少，map->odom 单步跳变从米级降到厘米级
POSE_GRAPH.optimize_every_n_nodes = 40
POSE_GRAPH.constraint_builder.sampling_ratio = 0.65
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.acceleration_weight = 1.1e2
POSE_GRAPH.optimization_problem.rotation_weight = 1.6e4
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
