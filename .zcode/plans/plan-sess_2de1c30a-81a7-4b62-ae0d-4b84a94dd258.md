# 3D 建图自打点 + 地面噪点 配置级修复方案（零代码）

## 根因（已用源码 + 几何计算定位）

**自打点为什么"min_range=0.72 没效果"：**
- 源码查证（cartographer 1.0.0 `local_trajectory_builder_3d.cc` AddRangeData）：3D 链路**确实**用 min_range，`range >= min_range()` 才保留 → 配置本身生效
- 但几何上自打点带是 **0.68~0.76m**（对向穹顶近面 0.68m，甲板后缘 0.76m）：0.72 只挡掉一部分，**0.72~0.76m 的边带存活**，观感就是"没效果"
- 另外 lua min_range 只作用于 Cartographer 内部；RViz 直接看 `/points2_1` 原始点云、nav2 代价地图（/scan）里的自打点依旧存在——需要在源头统一挡

**地面噪点：**
- 垂直 FOV 实际波束 -15°~-1° 共约 8 条向下射线，传感器离地仅 0.25m → 地面回波从 0.93m 铺到 14m
- 世界里障碍全是 3m 高墙（world_m.sdf 已确认），下方视野基本纯送噪点

## 改动清单（3 个配置文件，不动任何代码）

### 1. `src/simulated_chassis/urdf/three_wheel_chassis.xacro`（前后雷达两处 sensor，共 4 行）
- `<range><min>` 0.2 → **0.8**：gz GPU 雷达把 range-min 当渲染近裁剪面，0.8m 内自打点**根本不产生回波**（→inf→丢弃），点云/scan/所有下游统一干净；0.8 > 最远自打点 0.76m，与 2D 配置同值
- `<vertical><min_angle>` -0.1745 → **0.0**，`<max_angle>` 0.349 → **0.1745**（即 0°~+10°），samples 16 不变：实际波束 -5°~+5°，地面回波 ≥2.9m 仅 3 条稀疏波束。**FOV 中心保持 +5°（=实际 0° 水平），nav2 依赖的 /scan 水平波束不变**

### 2. `config/slam_3d_online.lua`（2 行）
- `min_range` 0.72 → **0.8**（与 xacro 对齐，第二道闸）
- `submaps.range_data_inserter.hit_probability` 0.55 → **0.65**（可选增强）：远处零星地面点多为单次命中，提门槛后不再显形；墙有双雷达成对累积多命中不受影响（2D 时代 hit=0.70 验证过同思路）

### 3. `config/localization_3d.lua`（2 行）
- `min_range` 0.72 → **0.8**；hit_probability 同步 0.65（与建图特征空间一致）

## 生效与验证

1. **无需 colcon build**（xacro 由 robot_state_publisher 启动时求值，lua 是符号链接直读）；需**重启整套仿真**（当前在跑的会话仍是旧配置，先 `scripts/clean_all_ros.sh`）
2. 验证顺序：
   - RViz 看 `/points2_1` 原始点云：机器人身上及 0.8m 内无回波点
   - `/scan_1` echo：无近距离自打回波（水平波束仍水平）
   - 起建图跑一小圈：子图地面噪点应稀疏化到 ≥2.9m 或基本消失，墙沿清晰
3. **需重建地图**：点云特征空间变了，旧 pbstream 不要混用（上位机 开始建图→遥控→保存）

## 取舍边界（选 -5°~+5° 推荐档的理由）

- 0.8m 内近距盲区：底盘 0.4×0.3m + nav2 inflation 本就把 0.8m 内当不可通行，无实际损失
- ±5° 在 10m 处垂直覆盖约 0.16~0.87m：矮于 0.15m 的障碍 3m 外看不见——当前世界无此类障碍；未来加低矮货架时把 FOV 下沿放宽回 -5°~+15° 即可（一行）
- 若建图时发现扫描匹配发散（低概率），同样放宽 FOV 下沿重试