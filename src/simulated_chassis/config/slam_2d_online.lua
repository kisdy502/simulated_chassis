-- Cartographer 2D 在线建图（双雷达水平扫描版）
--
-- 思路：平面地板 AGV 的建图/导航不需要 3D SLAM。Gazebo gpu_lidar 自带的
-- 2D LaserScan 输出（/scan_1、/scan_2）取垂直视场中心波束——两雷达各前倾
-- 5°、垂直中心 +5°，恰好合成水平波束（离地 0.25m，高于底盘顶 0.24m）：
--   - 波束平行于地面 → 地面回波在几何上不存在 → 2D 地图无灰点
--   - 波束越过自家底盘 → 四角自打点不存在（0.72m 处对方雷达穹顶由 min_range 挡掉）
--   - 2D SLAM 没有 z 轴 → z 漂移在数学上不存在
-- 3D 点云仍由 nav2 代价地图直接消费（其 min/max_obstacle_height 自带高度带）。
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
  num_laser_scans = 2,                    -- 双雷达：前 /scan_1，后 /scan_2
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,                   -- 不再消费 3D 点云
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

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

-- 2D 轨迹构建器：min_range 挡掉对方雷达穹顶(0.72m)等近场自打点
-- max_range 与传感器实际量程一致（xacro gpu_lidar range 0.2~30.0）：
-- Ignition 无回波发 inf（实测 /scan_1），inf > max_range 被 crop 丢弃、
-- 不会变成 30m 假命中，可放心对齐满量程（旧值 25 静默丢掉 25-30m 真实墙面）
TRAJECTORY_BUILDER_2D.min_range = 0.8
TRAJECTORY_BUILDER_2D.max_range = 30.0
-- 无回波(inf)射线长度置 0 = 整束丢弃（默认 5m）：细小物体（圆柱/立方体）
-- 在 2~8m 距离漏过波束（270束/270°，10m 处束间距 0.17m）时返回 inf，
-- 默认 5m 的 free 射线正好扫过近距离标记的格子——"靠近打上、走远消失"
-- 的直接推手。jzt 真机设为 max_range 是给动态障碍自愈留清除通道；仿真
-- 世界全静态，动态清除交给 nav2 obstacle 层。日后需要恢复清除改回 30.0。
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 0.
-- 前后雷达成对累积（各 270°，合成全向覆盖）：默认 1 = 单雷达半覆盖帧独立
-- 插入，扫描匹配抖动使标记发散成"模糊"。jzt 真机同类设置取 3。
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.use_imu_data = false         -- 轮式里程计的航向已足够
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90

-- 障碍物"粘性"：2D 概率栅格会用 miss 光线清除旧占据（新子图覆盖旧子图），
-- 这是 3D 投影时代"打上就不消失"与 2D 行为差异的根源。
-- hit 0.65→0.70：nav2 占据阈值是 0.65（value 65），单次远距命中恰好落在
-- 阈值上属刀尖平衡，再吃一条 miss 即跌破 →"模糊/消失"；0.70 让远距稀疏
-- 命中稳稳过线。miss 保持 0.45：近场噪声仍可清除，清除强度有限。
-- 注意层级：必须写到 probability_grid_range_data_inserter 里（对照
-- /opt/ros/humble/share/cartographer/configuration_files/trajectory_builder_2d.lua）。
-- 写浅一层（直接挂在 range_data_inserter 下）时 Lua 会静默建出新键，
-- cartographer 的 C++ 永不读取它，字典析构时 CHECK 崩溃：
-- "Key 'miss_probability' was used the wrong number of times"（建图秒退的根因）。
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability = 0.70
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability = 0.45

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.sampling_ratio = 0.65
POSE_GRAPH.constraint_builder.min_score = 0.60
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e5
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e5

return options
