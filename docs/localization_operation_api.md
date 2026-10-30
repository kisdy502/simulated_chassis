# 切图、重定位的异步执行结果

更新：2026-10-28。适用于本次新增接口后的 bridge；旧版本没有 operation_id。本文供 Robot Manager Service、调度与前端共同对接。

## 1. 受理不等于完成

仍使用原请求：

- /agv/load_map，类型 agv_bridge_v2_interfaces/srv/LoadMap，请求 map_name。
- /agv/relocalize，类型 agv_bridge_v2_interfaces/srv/Relocalize，请求 map_name、x、y、yaw。位置为 map 坐标米，yaw 为弧度。

响应新增 operation_id，原有 success / message / map_name 保留。success=true 表示操作已受理，不是最终完成。参数、地图、忙碌等接入错误立即 success=false，通常 operation_id 为空；若线程创建失败，可能携带 operation_id，可查询对应 FAILED 记录。

最终结论来自 /agv/localization_operation，或 /agv/get_localization_operation。不要仅靠 mode 变为 NAVIGATION、map_name 更新、网页刷新判断操作成功。

## 2. 上位机接入顺序

1. WebSocket 建立后订阅操作话题和常规 /agv/status。
2. 发送切图/重定位服务，记住本次 call_service 的 id。
3. 服务响应 success=false，立即按 message 告知失败；否则保存 operation_id，关联上层任务。
4. 收到相同 operation_id 的操作状态，更新执行阶段；只在 finished=true 时结束上层这次操作。
5. 接收顺序可能是“操作话题先到、服务响应后到”。收到 operation_id 后立即查询一次，补齐先到或丢失的状态。
6. 重连后按已保存的 operation_id 查询，不要自动重新发送切图/重定位。

新操作不会排队；已有定位操作未完成，或后台切换线程仍工作时，新请求会明确失败。上位机应避免重复提交。

## 3. 服务调用示例

切图：

```json
{"op":"call_service","id":"switch-request-1","service":"/agv/load_map","args":{"map_name":"map1025L"}}
```

重定位：

```json
{"op":"call_service","id":"reloc-request-1","service":"/agv/relocalize","args":{"map_name":"map1025L","x":1.0,"y":2.0,"yaw":0.5}}
```

受理响应示意（operation_id 由机器人生成，不要解析其内部格式）：

```json
{"op":"service_response","id":"switch-request-1","service":"/agv/load_map","result":true,"values":{"success":true,"message":"切图已受理，尚未完成；请按 operation_id 订阅 /agv/localization_operation 或查询操作结果","map_name":"map1025L","operation_id":"example-operation-1"}}
```

rosbridge 的 result=true 表示服务调用处理成功，values.success 才是业务受理结果。WebSocket id、机器人 operation_id、导航 command_id 是不同概念，不能混用。

## 4. 操作话题

- 名称：/agv/localization_operation。
- 类型：agv_bridge_v2_interfaces/msg/LocalizationOperation。
- 阶段变化即时发布，并约每墙钟秒重发最新操作；QoS reliable、transient_local、depth=1。
- 只追踪显式 load_map / relocalize；启动自动定位、后台自动恢复、建图保存不创建新的操作 ID。

```json
{"op":"subscribe","id":"localization-operations","topic":"/agv/localization_operation","type":"agv_bridge_v2_interfaces/msg/LocalizationOperation","throttle_rate":0}
```

| 字段 | 类型 | 含义 |
|---|---|---|
| header | std_msgs/Header | 本条发布的 ROS 时间；仿真暂停时不要用它衡量操作超时 |
| operation_id | string | 唯一操作标识，用于关联服务响应和上层任务 |
| operation_type | string | LOAD_MAP / RELOCALIZE |
| map_name | string | 本次请求的目标地图，不单独证明加载成功 |
| state | string | RUNNING / SUCCEEDED / FAILED / TIMED_OUT |
| stage | string | 当前阶段，失败/超时时保留最后阶段 |
| finished | bool | 是否已经有最终结论 |
| success | bool | 只有 SUCCEEDED 时为 true；RUNNING 的 false 不代表最终失败 |
| message | string | 当前进度或明确失败原因，不要依赖匹配中文文本分支 |
| pose_initialized | bool | 定位检查快照 |
| navigation_ready | bool | NavigationManager 就绪检查快照，独立于本次定位是否成功 |
| elapsed_sec | float64 | 单调时钟经过秒数，终态后固定 |

状态与阶段：

```text
RUNNING / ACCEPTED
  → STOPPING_LOCALIZATION
  → STARTING_LOCALIZATION
  → WAITING_LOCALIZATION
  → SUCCEEDED / FINISHED

任一未完成阶段 → FAILED 或 TIMED_OUT
```

阶段不保证每个都被客户端看到，必须接受直接收到终态。迟到的后台回调不会把已完成的 FAILED / TIMED_OUT 改写成成功；新操作也不会被旧 operation_id 的回调覆盖。

### 4.1 成功的准确含义

只有本次定位启动流程成功进入等待阶段、后台切换结束、现有 LocalizationMonitor 检查通过，才返回 SUCCEEDED。旧定位在启动阶段仍可用，不会仅凭它立即判成功。

此成功表示目标地图定位流程完成，判据使用现有 TF 可用性、新鲜度和连续稳定检查；不是新增的点云匹配精度或物理位置正确性保证。

Nav2 启动是另一阶段，SUCCEEDED 时 navigation_ready 仍可能为 false。调度若要立即发移动，应继续确认导航就绪；移动入口也会再次检查。最新记录的 pose_initialized / navigation_ready 会随心跳刷新，state / success / message / elapsed_sec 等最终结论保持不变。更早的历史记录保留各自最后快照。

### 4.2 失败和超时

失败原因包括旧进程未确认停止、定位启动失败、定位进程提前退出、后台异常、bridge 正常关闭前操作未完成等。失败不会再被当作成功回切 NAVIGATION。

localization_operation_timeout_sec 默认 600 秒，使用单调时间，包括停止、启动、等待定位的整个流程。Cartographer 大地图加载可能较慢，默认值为此留出余量。通用、UR5e、双臂 bridge launch 均可覆盖该参数。

```bash
# 在已有 bridge 启动参数后添加，单位为墙钟秒
localization_operation_timeout_sec:=600.0
```

超时只是本次操作的最终结论，不会擅自终止后台线程、重启机器人或自动重放请求。后台可能仍在退出或加载；新请求仍需通过切换互斥检查。FAILED / TIMED_OUT 后保持导航业务闸门，修正原因后显式重新切图或重定位；后台自动恢复不会把原结果变成成功。

## 5. 查询与断线恢复

服务：/agv/get_localization_operation，类型 agv_bridge_v2_interfaces/srv/GetLocalizationOperation。

请求：operation_id。传空字符串查询最近一次操作。

ROS2 命令行可直接检查最近记录：

```bash
ros2 service call /agv/get_localization_operation \
  agv_bridge_v2_interfaces/srv/GetLocalizationOperation "{operation_id: ''}"
```

响应：found、message、operation（完整 LocalizationOperation）。found=false 不是操作失败或成功，而是没有对应记录。

```json
{"op":"call_service","id":"query-switch-1","service":"/agv/get_localization_operation","args":{"operation_id":"example-operation-1"}}
```

```json
{"op":"service_response","id":"query-switch-1","service":"/agv/get_localization_operation","result":true,"values":{"found":true,"message":"操作记录已找到","operation":{"header":{"stamp":{"sec":0,"nanosec":0},"frame_id":""},"operation_id":"example-operation-1","operation_type":"LOAD_MAP","map_name":"map1025L","state":"SUCCEEDED","stage":"FINISHED","finished":true,"success":true,"message":"目标地图定位检查已通过；导航能否接任务请另查 navigation_ready 或导航就绪检查","pose_initialized":true,"navigation_ready":false,"elapsed_sec":12.3}}}
```

bridge 内存保留最近 128 条受理操作；话题只保留最新一条。WebSocket 重连但 bridge 未重启时，可以查回历史。bridge 重启或记录被淘汰后 found=false；本版本不做操作记录磁盘持久化，不要推断“重启前操作已经成功”。Robot Manager Service 应保存 operation_id 及已收到的终态，未知结果需要重新确认当前状态，由业务决定是否重试。

## 6. 升级与联调

服务响应增加字段，属于 ROS 接口定义变化。机器人接口包、bridge 以及直接使用这些 ROS 服务类型的客户端必须重新构建并重启；旧 JSON 客户端可继续发送原请求，但应新增读取 operation_id 及订阅/查询逻辑。

默认通用、UR5e、双臂启动白名单已放行新话题和服务。若部署时自定义 topics_sub_glob / services_glob，必须把它们加入实际白名单。

从工作区、加载 ROS 环境后按需构建：

```bash
colcon build --executor sequential --packages-select \
  agv_bridge_v2_interfaces agv_bridge_v2 \
  simulated_chassis_ur5e simulated_dual_arm
source install/setup.bash
```

无需全量删除或编译 Lightning；仿真本体不需要为这次功能重启。重新启动 bridge 才能加载新可执行文件与白名单；其托管定位和导航会随 bridge 生命周期受影响。

建议验收：

1. 正常切图和重定位：受理有 ID，最终成功只出现一次结论，页面无需刷新。
2. 非法地图：服务立即失败，不等待事件。
3. 启动失败：消息包含具体原因，不误报成功。
4. 无定位数据：到期限出现 TIMED_OUT，后续恢复不改原结果。
5. 重连：用原 ID 查询并恢复页面状态。
6. 重复提交：新请求明确失败，原操作继续。
7. Nav2 迟启动：定位成功与导航就绪分开呈现。

自动测试在隔离 ROS domain 中运行，不会向真实机器人提交切图或移动。真实地图端到端切换验收应安排在允许中断导航的时间。
