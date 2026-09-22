# AGV 运行状态：由 agv_nav_server 以 ~1Hz 周期发布
# 取代原本 HTTP POST /api/agv/handle_status 与 StatusUpdate bean
#
# 上位机（SpringBoot / 调度系统）通过 rosbridge 订阅：
#   {"op":"subscribe","id":"st","topic":"/agv/status","throttle_rate":1000}
# 因该 topic 使用 transient_local QoS，新接入的客户端会立刻收到最后一帧。

std_msgs/Header header

# AGV 唯一标识（agv_id 参数）
string agv_id

# 与 NavigationState 一致：IDLE / PRE_ROTATING / EXECUTING / BACKING_UP
# / END_ROTATING / COMPLETED / FAILED / CANCELLED；
# 另外，通过 /agv/set_control(action=stop) 暂停后固定上报 STOPPED
string state

# 电量百分比 0.0 ~ 100.0
float64 battery

# Cartographer / AMCL 纯定位是否已收敛，false 时上位机不应下发导航指令
bool pose_initialized

# 当前正在执行的指令，无任务时为空字符串
string active_command_id
string active_node_id

# 业务模式（由 /agv/load_map、/agv/start_mapping、/agv/save_map 驱动，与 state 正交）：
#   NAVIGATION   —— 正常导航定位（默认）
#   RELOCALIZING —— 正在重启定位节点加载新地图，收敛后自动回到 NAVIGATION
#   MAPPING      —— 在线建图中（start_mapping 进入，save_map 保存后退出）
string mode

# 当前加载的定位地图名（maps_dir 下三件套的文件名主干，不含扩展名）；
# 未记录 / 未知时为空
string map_name
