# 电量、充电接口与后端适配

整理日期：2026-10-27；电池功能版本日期：2026-10-26。本文对应新增电池功能的源码版本；机器人与 bridge 镜像同步更新后生效。
这是可以直接交给 Java/前端开发的接口说明，不需要上位机实现电量加减公式。

## 1. 本次需要对接什么

| 用途 | 接口 | ROS 类型 | 上位机操作 |
| --- | --- | --- | --- |
| 实时电量及充电状态 | /agv/status | agv_bridge_v2_interfaces/msg/AgvStatus | WebSocket subscribe，建议1Hz |
| 开始/停止充电 | /agv/set_charging | std_srvs/srv/SetBool | WebSocket call_service |
| 去充电点/去作业点 | /agv/follow_edge | agv_bridge_v2_interfaces/action/FollowEdge | 沿用现有导航 action |
| 可选：停车确认 | /odom | nav_msgs/msg/Odometry | 读取 twist.twist，建议10Hz |

当前 Compose 配置的 WebSocket 地址为 ws://机器人IP:9090；通用 launch 默认9090，以实际部署为准。
如果 Java 自己运行在另一个 Docker 网络，可通过宿主机映射地址 host.docker.internal:9090 访问；Java容器内的localhost指向Java容器自身。

## 2. 任务时序：先到位，再充电

```text
上位机“去充电”任务
  → 导航到充电点
  → 等待 action_result.values.success=true
  → 确认停车（建议稳定停止至少1秒）
  → /agv/set_charging(data=true)
  → 服务业务成功
  → 持续接收 CHARGING；电量逐渐增加
  → 100%时收到 FULL，充电通路仍可保持连接

上位机“去作业”任务
  → 直接下发原导航 action
  → bridge 如发现充电通路仍连接，先请求后端停止充电
  → 断开确认成功后才提交 Nav2 目标
  → 正常移动，电量逐渐减少
```

关键约定：

- 没有新增“一条请求自动去桩并充电”的 action；上位机充电任务负责串联导航和开始充电两步。
- 导航目标是充电点，不代表已经开始充电；收到任务或导航 ACCEPTED 都不能标记为充电中。
- 导航取消/失败时不得继续调用开始充电。
- 第一版仿真信任上位机选择的充电点，仅校验停车和近期移动命令，不验证充电点地理范围。
- 正常作业任务不要求上位机先额外调用停止充电；bridge已经负责该前置步骤。断开本身不移动机器人。
- 后端拒绝或超时，导航任务失败并返回原因，不自动重放，不会带着未确认的充电连接直接发起导航。
- 充满不自动下发作业任务或离桩，后续任务仍由上位机调度。

### 2.1 导航到充电点示例

下面的坐标、边ID仅作格式示例，必须替换成当前地图实际数据。

```json
{
  "op":"send_action_goal",
  "id":"charge_nav_001",
  "action":"/agv/follow_edge",
  "action_type":"agv_bridge_v2_interfaces/action/FollowEdge",
  "feedback":true,
  "args":{
    "command_id":"charge_task_001_move",
    "node_id":"DOCK_01",
    "edge_type":"STRAIGHT",
    "edge_id":"EDGE_TO_DOCK_01",
    "x":5.0,"y":2.0,"theta":1.57,
    "max_speed":0.2,
    "back_up":false,"reverse":false,
    "end_point":true,"step":0.1,"control_points":[]
  }
}
```

导航业务成功示例：

```json
{
  "op":"action_result",
  "id":"charge_nav_001",
  "action":"/agv/follow_edge",
  "status":4,
  "result":true,
  "values":{"success":true,"message":"导航到点完成","command_id":"charge_task_001_move","node_id":"DOCK_01"}
}
```

message仅示意，不能靠文案判断成功。导航成功后确认停车：平移速度 sqrt(vx*vx+vy*vy)，角速度取 abs(wz)。
当前仿真后端要求平移/转动均不超过0.02（m/s、rad/s），/odom最近2秒内有数据，且最近1秒无非零移动指令。
即使导航已成功，过早调用开始充电仍可能因停车条件未满足而失败；确认停车后可有限次重试，禁止无限循环。
其他导航模式与参数规则见 [导航接口总文档](rosbridge_integration.md) 和 [FREE自由导航](free_navigation_api.md)。

## 3. 开始/停止充电服务

### 3.1 参数

| 字段 | 类型 | 要求 |
| --- | --- | --- |
| op | string | 固定 call_service |
| id | string | 每次请求使用唯一ID，用于关联响应；不是后端持久化去重键 |
| service | string | 固定 /agv/set_charging |
| args.data | boolean | true=建立充电连接；false=停止充电并断开充电通路 |
| timeout | number | 建议5.0秒，是rosbridge调用超时；后端确认等待默认为2秒 |

data必须显式传true/false，不要传null、字符串或0/1。ROS的bool字段被省略时默认false，会变成停止充电，不是“保持不变”。

### 3.2 开始充电

```json
{"op":"call_service","id":"charge_start_001","service":"/agv/set_charging","args":{"data":true},"timeout":5.0}
```

成功响应示例：

```json
{"op":"service_response","id":"charge_start_001","service":"/agv/set_charging","result":true,"values":{"success":true,"message":"仿真充电连接已建立，电量将随仿真时间增加"}}
```

仍有移动任务时的业务失败示例：

```json
{"op":"service_response","id":"charge_start_001","service":"/agv/set_charging","result":true,"values":{"success":false,"message":"不能开始充电：存在移动任务；请等待导航到充电点成功并停车后重试"}}
```

### 3.3 主动停止充电

```json
{"op":"call_service","id":"charge_stop_001","service":"/agv/set_charging","args":{"data":false},"timeout":5.0}
```

```json
{"op":"service_response","id":"charge_stop_001","service":"/agv/set_charging","result":true,"values":{"success":true,"message":"仿真充电连接已断开，已转为放电"}}
```

没有充电时重复停止也是允许的。重复开始不会叠加充电速率，但仍要满足停车、数据有效等条件。
该接口是设置连接状态，不是具有任务ID、排队、取消、进度回调的长时间充电action。

### 3.4 必须区分两层成功

- 外层result=true：服务调用正常完成；仍必须检查values.success。
- values.success=true：机器人侧业务处理成功。
- 外层result=false：rosbridge转换/调用错误，values可能是字符串，不能强制按对象解析。

例如显式传data:null可能返回：

```json
{"op":"service_response","id":"charge_start_001","service":"/agv/set_charging","result":false,"values":"msg is not a primitive type, but a <class 'NoneType'>"}
```

上位机收不到响应或WebSocket断开，不代表充电请求没有执行。重新连接后先获取状态，不要立即标记为失败后继续移动。
只有机器人内部的充电后端确认超时/通信异常，才会设置“控制结果未确认”保护；普通上位机网络超时不一定触发该保护。
若收到未确认保护错误，修复后端后显式调用停止充电，成功确认后再重新提交原作业任务。

## 4. 实时状态字段

订阅请求：

```json
{"op":"subscribe","id":"sub_agv_status","topic":"/agv/status","type":"agv_bridge_v2_interfaces/msg/AgvStatus","throttle_rate":1000,"queue_length":1}
```

充电中上报示例（新增字段已包含）：

```json
{
  "op":"publish","topic":"/agv/status",
  "msg":{
    "header":{"stamp":{"sec":100,"nanosec":0},"frame_id":"map"},
    "agv_id":"AGV002","state":"IDLE",
    "battery":65.0,"battery_state":"CHARGING",
    "charging":true,"charge_connected":true,"battery_data_valid":true,
    "pose_initialized":true,"active_command_id":"","active_node_id":"",
    "mode":"NAVIGATION","map_name":"map1025L","slam_alive":true,"map_age_sec":1
  }
}
```

| 字段 | 类型/单位 | 语义 |
| --- | --- | --- |
| battery | number，0~100% | 业务电量百分比，例如65表示65%，不是0.65 |
| battery_state | string | 电池状态枚举，见下表；与导航state独立 |
| charging | boolean | 最近有效电池遥测是否报告正在充电；FULL时通常false |
| charge_connected | boolean | 最近已知的充电通路连接状态；FULL时可以仍为true |
| battery_data_valid | boolean | 最近电池遥测是否新鲜且电量有效；默认有效期5秒 |
| state | string | 既有导航状态；不增加CHARGING等电池枚举 |

| battery_state | 上位机展示与处理 |
| --- | --- |
| DISCHARGING | 放电中；静止也缓慢耗电 |
| CHARGING | 充电中；电量随时间增加 |
| FULL | 已充满；不要据此判断已离桩/已断开 |
| NOT_CHARGING | 后端报告未在充电；连接状态另看charge_connected |
| UNKNOWN | 数据无效/过期，或充电控制结果未确认；不得据此自动认为可移动 |

注意：

- battery_data_valid=false时，battery可能是上次值或配置回退值，不能继续当作当前电量，也不能把无效值强制显示成0%。
- UNKNOWN与charging可以暂时并存：battery_state还考虑充电控制结果是否确认，而charging只反映最近有效电池遥测。
- charge_connected是最近已知值，没有独立时间戳；数据无效或控制未知时，不能仅凭false判断“已安全断开”。
- 服务响应和下一次电池上报之间可能有约一个发布周期的延迟，不要要求同一瞬间所有字段已经同步。
- 建议以本地接收时间监测状态流。状态数秒未到时显示离线/未知并暂停自动任务；header.stamp可能是仿真时间，暂停仿真时不会增长。
- 不要等电量0%才调度去充电。0%会拒绝新导航，并取消正在执行的导航；低电量预警和提前回桩阈值由上位机配置，机器人侧不会自动寻桩。

## 5. 失败原因与上位机修复方式

错误文案用于展示和诊断，不是稳定的错误码；程序主要依据success/status和本地任务阶段处理。

| 场景/消息要点 | 上位机处理 |
| --- | --- |
| 不能开始充电：存在移动任务 | 等待导航成功、停车后再开始充电 |
| 仍在运动、最近收到移动指令或/odom超时 | 确认速度已为零，停止遥控输入，检查里程计后有限重试 |
| 电池数据无效或过期 | 显示未知，检查机器人电池节点/驱动，不继续自动导航 |
| 电量已耗尽 | 先恢复充电；不要反复重发移动任务 |
| 电池充电状态UNKNOWN | 检查驱动的状态上报；如果是控制未确认，执行显式停止充电流程 |
| 充电后端服务不可用 | 检查机器人端/电池驱动服务及配置，不修改上位机坐标来绕过 |
| 充电后端2秒内未确认/通信异常 | 结果未知；恢复后端后显式停止充电，确认成功再重新下发任务 |
| 移动前停止充电失败，未发送导航目标 | 作业任务本次失败；处理后端问题后由调度重新下发，不等待自动重放 |

移动前检查失败使用原导航action失败结果返回，例如：

```json
{"op":"action_result","id":"work_nav_001","action":"/agv/follow_edge","status":6,"result":true,"values":{"success":false,"message":"移动前停止充电失败，未发送导航目标：充电设备拒绝断开","command_id":"work_task_001_move","node_id":"WORK_01"}}
```

取消“去充电点”的导航仍使用既有cancel_action_goal；导航取消不等价于停止已经建立的充电连接。
取消一个已进入充电阶段的上位机任务时，应显式调用停止充电；若业务希望取消任务但继续留桩充电，需要在上位机明确区分该策略。

## 6. 重连、版本与联调验收

- 新客户端收到状态时，应先检查四个新增字段是否存在。缺字段说明机器人镜像/消息版本未更新，不能按false/100%默认值解释为正常。
- 仿真机器人与bridge两个镜像需同步更新。仅重启旧镜像不能获得功能；原生ROS消息消费者需使用更新后的AgvStatus重新构建。
- bridge重启不重置模拟电量；电池节点重启恢复SOC，但不会自动恢复充电连接。重连后以实际状态重新建立任务阶段，不盲目恢复“充电中”。
- 同一机器人只使用一个电池数据源，不能同时启动模拟电池和真机BMS发布器。
- 上位机不直接调用私有/battery/set_charging，也不发布伪造/battery_state；统一通过/agv接口。

联调清单：

1. 空闲时battery缓慢下降，移动时下降更快；原地转动也按运动计算。
2. 还在导航时开始充电失败，失败原因能展示。
3. 到充电点并停车后开始充电，状态和电量持续上报。
4. 充满变FULL，仍能显示连接中。
5. 充电中直接下发作业，机器人先停止充电再执行导航。
6. 停止充电被拒绝/超时，不执行移动，任务能明确失败。
7. 电池数据过期、WebSocket断线、bridge重启后，界面不会继续显示“数据正常”。
8. 电池节点重启保留已保存电量，而非统一重置为100%，也不自动建立充电连接。

仿真默认按仿真时间计算：充电每1分钟增加1个百分点；静止每5分钟减少1个百分点；移动/原地转向每2分钟减少1个百分点（移动耗电不再叠加静止耗电）。暂停仿真不涨/掉电。
需要快速演示时由机器人侧调整模拟速率，不由上位机修改收到的电量。
仿真参数、持久化和真机BMS适配契约见本文第7、8节。

## 7. 仿真配置与持久化

包 battery_simulator，C++可执行程序 battery_simulator_node。
底盘 simulation launch 默认启动一个电池节点，组合UR5e仿真同样沿用该节点。
通过 battery_simulation:=false 可关闭模拟后端，用于接入外部电池驱动。
节点在机器人容器运行；bridge重启不改变它的电量或充电状态。

配置文件 battery_simulator/config/battery.yaml（启动时读取，修改后需要重启电池节点）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| initial_soc | 80.0 | 首次启动电量；存在持久化文件时优先恢复文件 |
| idle_discharge_pct_per_hour | 12.0 | 静止每5仿真分钟减少1个百分点 |
| moving_discharge_pct_per_hour | 30.0 | 移动/原地转向每2仿真分钟减少1个百分点，非叠加静止耗电 |
| charge_pct_per_hour | 60.0 | 充电每1仿真分钟增加1个百分点 |
| capacity_ah | 40.0 | 用于模拟容量/电流遥测 |
| nominal_voltage | 48.0 | 模拟固定电压，非真实电池电压模型 |
| state_file | ~/.ros/battery_simulator/soc.state | SOC持久化文件，空字符串禁用持久化 |

电量按时间差积分，使用仿真时钟。暂停仿真不改变电量；时钟后退或单次前跳超过30秒时重置计时基准，不进行巨量补扣/补充。
遥测按wall clock每秒发布，SOC每10秒及优雅退出时保存；异常断电最多丢失最近10秒左右的变化。离线期间不补算耗电。
重启电池节点恢复SOC，但不恢复充电连接，避免重启后未经确认自动充电。
Docker已有 /root/.ros 持久化挂载，文件对应 runtime/simulation/ros/battery_simulator/soc.state。
测试快速涨电可将 charge_pct_per_hour 设置为3600，即每仿真秒增加1个百分点；这只是演示速率。

## 8. 真机BMS适配契约

```text
仿真：BatterySimulator（时间积分） ─┐
                                 ├─ 标准遥测/控制接口 → BatteryGateway → 上位机
真机：BMS/充电驱动适配器 ──────────┘
```

需要替换的是仿真后端，不是简单在bridge里把加减公式换成读串口。
真机适配器读取BMS的SOC、电压、电流和状态，发布：

- /battery_state：sensor_msgs/BatteryState，percentage严格为0~1；无效使用NaN/UNKNOWN，不能伪造100%。
- /battery/charging_connected：std_msgs/Bool，反馈充电通路是否仍连接。
- /battery/set_charging：std_srvs/SetBool，负责实际充电控制及硬件联锁。

后端 success=true 必须表示控制已完成并确认，而非仅将命令加入硬件队列。开始充电需确认停车/接触/允许充电；停止充电需确认电流切断，并完成硬件要求的释放，才允许移动。
若真机是手插线缆、需要机械脱钩或专门离桩动作，不能只关闭充电电流就放行导航；适配器必须在可安全移动前保持失败/未确认。
模拟节点对运动命令的自动断充电只是仿真兜底，不能复制为真机“带线启动后再断开”的控制逻辑。
保留复用：BatteryGateway的状态汇总、数据新鲜度检查、作业前断充电确认、空电量任务保护，以及上位机接口。

bridge参数：require_battery_state（通用入口默认false，组合仿真/Docker默认true）、battery_state_timeout_sec（默认5秒）、charging_backend_service（默认 /battery/set_charging）。
电池数据曾经接入后若变为过期，即使require_battery_state=false，也会拒绝新移动任务。0%时取消正在执行的导航，但不模拟电机断电，不代替真机底盘安全控制。
