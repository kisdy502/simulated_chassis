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
