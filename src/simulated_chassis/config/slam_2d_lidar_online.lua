-- Cartographer 2D 在线建图（2D 雷达版机器人 three_wheel_chassis_2d.xacro）
--
-- 与 slam_2d_online.lua（3D 雷达的水平扫描版）的差异：
--   传感器换成真 2D 雷达（左前/右后斜对角、底盘角镂空低位安装），
--   扫描面离地 0.105m、水平无俯仰、量程 0.10~25m（见 xacro）。
--   两雷达 90° 盲区朝向机体中心：轮组/转向节/对角雷达互不可见，
--   不再需要 min_range=0.8 滤"对方雷达穹顶 0.72m"（3D 版前后布局的遗留），
--   min_range 放宽到 0.15 —— 低位雷达近场障碍（≥0.15m）也能进图。
--   其余（成对累积、inf 置 0、hit/miss 概率）与 3D 版 2D 建图同源。
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",
  odom_frame = "odom",
  provide_odom_frame = false,
  publish_frame_projected_to_2d = true,
  -- tracked_pose_tf_node 同时刻重锚定 map→odom，控制器发布 odom→base_footprint。
  use_pose_extrapolator = true,
  publish_to_tf = false,
  publish_tracked_pose = true,
  use_odometry = true,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 2,                    -- 双雷达：左前 /scan_1，右后 /scan_2
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
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

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 2D 轨迹构建器：雷达凸出底盘角安装，几何上无自打点；min 0.15 仅留保险余量
-- （gpu_lidar range min 0.10，角点擦边杂点在 0.10~0.15 之间被这里裁掉）。
-- max_range 与传感器实际量程一致（xacro gpu_lidar range 0.10~25.0）：
-- Ignition 无回波发 inf，inf > max_range 被 crop 丢弃，可放心对齐满量程。
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 25.0
-- 无回波(inf)射线长度置 0 = 整束丢弃（默认 5m）：细小物体在 2~8m 距离漏过
-- 波束（540束/270°，10m 处束间距 0.08m）时返回 inf，默认 5m 的 free 射线
-- 正好扫过近距离标记的格子——"靠近打上、走远消失"的直接推手。
-- 仿真世界全静态，动态清除交给 nav2 obstacle 层。
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.
-- 左前/右后雷达成对累积（各 270°，FOV 并集全向 360°、对角重叠约 90°×2）：
-- 单雷达半覆盖帧独立插入会因扫描匹配抖动使标记发散成"模糊"。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.use_imu_data = false         -- 轮式里程计的航向已足够
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 障碍物"粘性"：与 3D 版 2D 建图同源。
-- hit 0.65→0.70：nav2 占据阈值是 0.65，单次远距命中恰好落在阈值上属刀尖
-- 平衡，再吃一条 miss 即跌破 →"模糊/消失"；0.70 让远距稀疏命中稳稳过线。
-- miss 保持 0.45：近场噪声仍可清除，清除强度有限。
-- 注意层级：必须写到 probability_grid_range_data_inserter 里（对照
-- /opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua）。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.65
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
