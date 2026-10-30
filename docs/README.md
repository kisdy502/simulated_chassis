# 项目文档索引

接口与专题说明统一维护在本目录；各包README保留包级使用入口。更新接口时修改对应专题，并同步主协议中的引用。

## 上位机接口

- [切图与重定位操作结果](localization_operation_api.md)：操作 ID、阶段、成功/失败/超时通知与断线结果查询。
- [WebSocket / rosbridge 主协议](rosbridge_integration.md)：连接、状态、导航、地图与服务调用。
- [自由绕障到点](free_navigation_api.md)：FREE导航参数与失败处理。
- [3D地图、URDF与mesh](map_model_api.md)：HTTP与WebSocket参数、响应、URDF读取排查。
- [相机与按需拍照](camera_api.md)：触发、元数据、预览和展示能力。
- [机械臂控制](arm_move_integration.md)：动作参数与结果。
- [通用双指夹爪](gripper_api.md)：末端工具切换、开合服务、实际开口和控制器状态。
- [电量与充电](battery_api.md)：上位机完整契约、充电流程、仿真配置、持久化与真机BMS适配。

## 运维与架构

- [Bridge 业务、源码与 C++ / ROS2 导读](bridge_source_guide.md)：启动、导航检查与状态机、取消与恢复、地图定位、电池，以及智能指针、回调、模板、线程和 ROS2 API。
- [建图与定位](mapping_and_localization.md)
- [Lightning 建图重影排查](lightning_mapping_diagnostics.md)：地图/位姿坐标一致性、扫描时间戳和绘图并发。
- [Lightning与Cartographer](lightning_vs_cartographer.md)
- [复合机器人架构](composite_robot_architecture.md)
- [Docker部署](../DOCKER_DEPLOY.md)
- [根目录README：按包清理与编译](../README.md)

原电池接口与充电对接文档已合并为 `battery_api.md`；原URDF 404修复说明已合并到 `map_model_api.md` 第3.1节。`src/agv_bridge_v2/docs/` 不再维护文档副本。
