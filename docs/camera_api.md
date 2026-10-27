# 腕部相机、按需拍照与照片展示

核对日期：2026-10-23。依据 `wrist_camera_node.py`、`wrist_camera.yaml`、
`wrist_camera.xacro` 和 UR5e 组合 bridge 白名单。

## 当前工作方式

Gazebo相机为 `always_on=true`，默认640×480、10Hz持续出图；ROS节点缓存最新帧并转发。
**连续出图不等于连续保存照片**：只有调用 `/wrist_camera/take_photo` 才写PNG。
服务等待请求处理开始之后的新帧，不直接控制传感器单次曝光。
目前没有夹爪/机械臂动作自动触发拍照的联动，需要任务系统显式调用。

“需要时保存一张”已有；“平时不渲染、不采集，收到指令才曝光”尚未实现。
取消上位机图像订阅不停止传感器；YAML的 `enabled:false` 是加载模型时关闭相机，
不是运行时触发开关。

## 拍照服务（已实现）

原生 WSL 连接 `ws://HOST:9090/`；从宿主机访问 Docker 机器人连接
`ws://HOST:9090/`；同 Docker 网络的后端连接 `ws://agv-bridge:9090/`。
相机没有独立 HTTP/WebSocket 监听端口，拍照复用 rosbridge。
ROS类型 `std_srvs/srv/Trigger`。UR5e组合bridge已放行；
纯底盘通用bridge不默认放行该服务。

```json
{"op":"call_service","id":"photo-42","service":"/wrist_camera/take_photo",
 "args":{},"timeout":10.0}
```

请求无业务参数，不能指定文件名、曝光、分辨率、相机ID或格式。
id仅关联WS响应，不是照片业务ID，也没有幂等去重。
timeout是rosbridge等待秒数，应大于相机采帧超时，并预留PNG写盘时间。

成功示例（路径随环境变化）：

```json
{"op":"service_response","id":"photo-42","service":"/wrist_camera/take_photo",
 "result":true,"values":{"success":true,
 "message":"/root/.ros/wrist_camera/photos/photo_1234_567000000_xxxxxx/image.png"}}
```

无新帧失败示例：

```json
{"op":"service_response","id":"photo-42","service":"/wrist_camera/take_photo",
 "result":true,"values":{"success":false,
 "message":"No new camera frame before timeout; check simulation, rendering and bridge"}}
```

外层result表示调用层结果，是否拍照成功要看values.success。
保存失败时message为 `Photo save failed: ...`，本次新建目录会被清理。
服务回调串行处理；传感器回调在另一回调组，允许等待期间接收新帧。
只有收到新帧且图像ROS时间晚于服务处理起点才保存；仿真暂停、渲染失效、
只有旧队列帧或时间源不一致时会超时失败。起点不是上位机点击时刻。
WS超时不代表相机端没保存，直接重试可能产生多张照片。

## 文件与配置

每次创建独立目录，含image.png和metadata.json。

| 配置 | 默认 | 含义 |
| --- | --- | --- |
| output_dir | `~/.ros/wrist_camera/photos` | ROS参数；simulation launch的photos_dir转发至此 |
| capture_timeout_sec | 5.0 | ROS参数，范围(0,60]，启动读取；launch无同名转发参数 |
| enabled | true | wrist_camera.yaml，模型加载时开关 |
| width / height | 640 / 480 | 同上，像素 |
| update_rate | 10.0 | 同上，持续出图Hz |
| horizontal_fov | 1.0471975512 | 同上，弧度，约60度 |
| near / far | 0.02 / 20.0 | 同上，米 |

metadata包含stamp（sec/nanosec）、frame_id、width、height、camera_info。
图像stamp是ROS时间，仿真时不是现实日期；frame为ur5e_camera_optical_frame。
camera_info可为null，否则含自己的stamp、distortion_model、d/k/r/p。
内参取最近一份匹配分辨率的数据，不保证图像/内参严格同步。
坐标链：ur5e_tool0 → ur5e_camera_link → ur5e_camera_optical_frame；光学轴Z前、X右、Y下。

## 上位机展示：已有能力与缺口

实时预览可订阅：

```json
{"op":"subscribe","id":"camera-preview","topic":"/wrist_camera/image_raw",
 "type":"sensor_msgs/msg/Image","throttle_rate":1000,"queue_length":1}
```

Image字段：header、height、width、encoding、is_bigendian、step、data。
这是原始像素，不是PNG/JPEG；需按rosbridge字节数组传输规则、encoding和step解码。
内参话题为 `/wrist_camera/camera_info`（sensor_msgs/msg/CameraInfo）。
节流只限制客户端转发，不关闭相机；不需要预览就不订阅，用完发送unsubscribe。

**已拍照片目前没有HTTP下载接口**：响应message是机器人文件路径，不是浏览器URL。
8089只提供地图，8090只提供ROS包资源，都不是照片服务器。
尚未实现照片列表、下载、删除、拍照结果图片话题或service内返回base64图片。
WSL文件在运行仿真用户的output_dir；Docker默认在仿真容器
`/root/.ros/wrist_camera/photos/`，宿主映射为
`runtime/simulation/ros/wrist_camera/photos/`，不在bridge容器。
后端能访问该目录时可读PNG并通过上位机自己的HTTP提供；独立远程部署仅凭路径不能展示。

## 末端按需拍照的调用顺序

1. 任务系统控制机械臂/夹爪，等待动作成功，按工艺要求等待稳定。
2. 显式调用一次take_photo，检查外层result和内层success。
3. 后端读取本次PNG展示；无法访问目录则需要先补照片下载能力。
4. 关联业务任务号、机械臂位姿、图像时间戳和照片记录。

实时流中的任意帧不能替代本次拍照结果，两者未必是同一张。
当前运动、夹爪和拍照不是已有的原子联动操作。
建议后续补充（尚未实现）：带幂等请求ID的拍照契约，返回
photo_id/image_url/metadata_url/stamp/width/height，配套只读PNG和metadata下载。
为兼容旧客户端，不直接改变现有Trigger.message路径格式。
严格单次曝光需要传感器驱动触发模式，不能只改保存逻辑或将update_rate设为零。

## 验证范围

本次只核验代码和补文档，未实际拍照、移动机械臂、改配置或重启。
已有ROS测试 `simulated_chassis_ur5e/test/test_wrist_camera.py` 覆盖无帧超时、旧帧拒绝、
请求后新帧保存PNG和metadata；另有Fortress集成测试。

相关文档：[地图/模型](map_model_api.md)、[机械臂](arm_move_integration.md)、
[WebSocket](rosbridge_integration.md)。
