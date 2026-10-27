# 3D 地图、URDF 与地图业务接口

核对日期：2026-10-23。依据当前工作区的 `map_3d_server.cpp`、`assets.cpp`、
`urdf_mesh_server.cpp`、launch 配置、`.srv/.action/.msg` 和本机 Humble rosapi 实现。
`HOST` 为机器人主机；示例中的地图名仅作示例。以下是机器人侧接口，
上位机 Java 的 `/api/v1/map-3d/*`、`/api/v1/robot/urdf` 属于另一层代理，不由本仓库实现。

相机触发拍照、实时预览、结果文件和照片展示缺口另见 [相机接口](camera_api.md)。

## 1. 接入地址与启动条件

下面的 9090/8089/8090 示例是原生 WSL 监听地址或 Docker 内部地址。
从 Windows/浏览器访问 Docker 机器人时，对应端口为 **9090/8089/8090**；
同 Docker 网络的后端仍使用 `agv-bridge:9090/8089/8090`。
完整访问矩阵和代理配置见 [Docker 部署说明](../DOCKER_DEPLOY.md#2-对外端口与上位机接入)。

| 地址 | 用途 | 启动条件 |
| --- | --- | --- |
| `ws://HOST:9090/` | rosbridge：业务 service/action、状态订阅、URDF 参数查询 | 通用 bridge |
| `http://HOST:8089/maps/<name>/status` | 已保存地图三维导出状态 | `enable_map_3d:=true`，默认开启 |
| `http://HOST:8089/maps/<name>/cloud.ply` | 下载三维点云 | 同上，导出 ready 后可下载 |
| `http://HOST:8090/<package>/<path>` | 下载 URDF 引用的 mesh/纹理等包资源 | UR5e/双臂组合 bridge 的 `mesh_server:=true` |

通用 `agv_rosbridge.launch.py` 不启动 8090 服务；纯底盘要用该服务时可单独启动：

```bash
ros2 run agv_bridge_v2 urdf_mesh_server --host 0.0.0.0 --port 8090
```

Docker 中 `WEBSOCKET_PORT` / `MAP_3D_PORT` / `URDF_MESH_PORT` 只改宿主机映射，
默认 9090 / 8089 / 8090，容器内部固定 9090 / 8089 / 8090。
ROS launch 通过 `map_3d_port`、组合 profile 的 `mesh_server_port` 改服务端口。

## 2. 3D 地图 HTTP 契约

### 2.1 查询状态

`GET /maps/{name}/status`，无请求体。`name` 是地图目录/文件主干名，不带扩展名，
UTF-8 字节长度为 1～128，不能含 `..`、`/`、反斜杠或 NUL；建议沿用 `/agv/list_maps` 返回的名字。查询串不作为业务参数使用，
没有 HTTP `voxel`、`force`、`refresh` 等请求参数。

```bash
curl -i http://HOST:8089/maps/map1022Pro/status
```

成功返回 `200`，`Content-Type: application/json; charset=utf-8`，`Cache-Control: no-store`。

```json
{
  "name": "map1022Pro",
  "status": "ready",
  "frame_id": "map",
  "version": "opaque-source-version",
  "voxel_size": 0.1,
  "point_count": 250878,
  "size_bytes": 3010677
}
```

| 字段 | 类型 | 出现条件及含义 |
| --- | --- | --- |
| `name` | string | 地图名；字段是 `name`，不是 `map_name` |
| `status` | string | `generating` / `ready` / `failed` |
| `frame_id` | string | 固定 `map`，米，Z 向上 |
| `version` | string | 源文件及导出配置的版本标识；仅比较是否变化，不解析内部格式 |
| `voxel_size` | number | 服务端配置的降采样体素尺寸，米 |
| `point_count` | integer | ready 时的顶点数 |
| `size_bytes` | integer | ready 时整个 PLY 文件字节数，含文件头 |
| `message` | string | failed 时的原因；generating 等待源文件稳定时也可能出现 |
| `retry_at` | number | failed 时的下一次允许重试时间，Unix 墙钟秒；不是 ROS 仿真时间 |

生成中示例：

```json
{"name":"map1022Pro","status":"generating","frame_id":"map",
 "version":"opaque-source-version","voxel_size":0.1,
 "message":"Waiting for a stable source file"}
```

失败示例（`message` 具体文本随故障而变）：

```json
{"name":"map1022Pro","status":"failed","frame_id":"map",
 "version":"opaque-source-version","voxel_size":0.1,
 "message":"export failed","retry_at":1792745000.0}
```

注意：**导出 failed 的状态查询仍返回 HTTP 200**，要检查 JSON 的 `status`。
查询可能排队触发后台生成，不是完全无副作用的静态状态读取。

### 2.2 下载点云

`GET /maps/{name}/cloud.ply`，无请求体，成功 `200`，
`Content-Type: application/octet-stream`，`Content-Length` 是实际大小，`Cache-Control: no-cache`。

```bash
curl --fail -o map1022Pro.ply http://HOST:8089/maps/map1022Pro/cloud.ply
```

响应是 **PLY 文件字节**，不是 JSON。文件为 `binary_little_endian 1.0`，
每个顶点依次为 float32 `x/y/z`，每点 12 字节；不含颜色、法线和三角面。
前端应按 PLY 格式解码，不能把整个响应当作纯 XYZ 数组（前面还有文本文件头）。
当前校验限制为 1～5,000,000 点。

### 2.3 方法与错误

两条路由均支持 `HEAD`（同类状态和长度信息，无正文）；`OPTIONS` 返回 `204`。
所有响应带 `Access-Control-Allow-Origin: *`；预检允许 `GET, HEAD, OPTIONS` 和 `Content-Type`。

| HTTP 状态 | 场景 | 响应体 |
| --- | --- | --- |
| 200 | 状态查询成功或下载成功 | 状态 JSON / PLY |
| 400 | 路径解析/名称等无效 | `{"error":"具体原因"}` |
| 404 | 未知路由或所选后端找不到地图源 | `{"error":"具体原因"}` |
| 409 | 下载时尚未 ready、源文件变化或结果文件变化 | `{"error":"3D map is not ready"}` 或其他原因 |
| 405 | 不支持的方法，如 POST | `{"error":"Only GET, HEAD and OPTIONS are supported"}` |
| 500 | 请求处理异常 | `{"error":"3D asset request failed"}` |

没有上传/删除/列目录 HTTP API，也未实现 Range 分段、ETag 或条件下载协议。
建议保存完成后以 2～5 秒间隔查询状态，ready 后下载；遇到 409 重新查状态。
`/agv/list_maps` 的二维可加载地图列表不保证每项都有当前后端需要的三维源。

### 2.4 数据来源与服务端参数

Cartographer：`maps/<name>/<name>.pbstream`，兼容 `maps/<name>.pbstream`；
Lightning：`maps/<name>/global.pcd`。bridge 根据 `slam_algorithm` 固定选择 pbstream 或 pcd。
三维数据对应保存结果，不是实时点云流，也不是三维路径规划接口。

| CLI 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `--maps-dir` | 必填 | 地图根目录；同一根目录不允许重复启动服务 |
| `--address` / `--port` | `0.0.0.0` / `8089` | HTTP 监听地址和端口 |
| `--source-format` | `auto` | `auto` / `pcd` / `pbstream`；auto 优先嵌套 pbstream、平铺 pbstream、pcd |
| `--voxel` | `0.1` | 米，范围 `0.001 <= voxel < 10` |
| `--settle-seconds` | `2` | 源文件签名稳定等待时间，>=0 |
| `--scan-seconds` | `10` | 后台扫描间隔，>0 |
| `--export-timeout` | `300` | 导出超时秒数，>0 |
| `--retry-seconds` | `30` | 导出失败重试间隔，>=0 |
| `--exporter` / `--exporter-pcd` | 可执行文件同目录下的导出器 | pbstream / pcd 转换程序路径 |

这些是启动参数，不是 HTTP 查询参数。通用 bridge 目前只转发根目录、端口、后端和导出器路径，
其他项目使用上述默认值。源文件变化会自动重导出；生成串行进行，输出原子替换。

## 3. 获取当前完整 URDF（WebSocket）

**当前展开后的 URDF 来自 `/robot_state_publisher` 的 `robot_description` 参数。**
8090 只按包路径读取文件，不执行 xacro，不负责生成当前机器人完整 URDF。

在 `ws://HOST:9090/` 发送：

```json
{"op":"call_service","id":"urdf-1","service":"/rosapi/get_param",
 "args":{"name":"/robot_state_publisher:robot_description","default_value":"\"\""},
 "timeout":10.0}
```

| 参数 | 类型 | 说明 |
| --- | --- | --- |
| `id` | string | 请求关联 ID，响应回传 |
| `args.name` | string | 本机 Humble rosapi 格式为 `节点全名:参数名`，注意冒号 |
| `args.default_value` | string | 可选的 JSON 编码默认值；示例表示 JSON 空字符串 |
| `timeout` | number | rosbridge 服务调用等待时间，秒；不是 robot_description 的 ROS 参数 |

示意响应：

```json
{"op":"service_response","id":"urdf-1","service":"/rosapi/get_param",
 "result":true,"values":{"value":"\"<robot name=\\\"demo\\\">...</robot>\""}}
```

`values.value` **本身还是一段 JSON 编码的字符串**。客户端先解析 WebSocket 信封，
再对 `values.value` 做一次 JSON 解码，才得到 XML：

```javascript
const envelope = JSON.parse(wsText);
const urdfXml = JSON.parse(envelope.values.value);
```

`result=true` 只表示服务正常响应；参数缺失可能返回编码后的默认空串。
还需检查解码结果非空且包含 `<robot`，并处理超时、白名单拒绝、无响应及缺少 `value`。
不要发送旧文档中的 `args:{node:...,param:...}`，这不符合本机 `GetParam.srv`。
具体服务名/类型可通过 ROS 图查询确认，节点改名时也须修改 `name`。

### 3.1 URDF读取失败与自检

请求的 `name` 必须使用 `/robot_state_publisher:robot_description`，不能替换为裸参数名或斜杠形式。上位机代理返回404时，先检查机器人服务的实际返回值，再检查代理转换；机器人接口本身是WebSocket服务，不是上位机的HTTP端点。

在机器人所在ROS环境中自检（存在命名空间时按实际节点名调整）：

```bash
ros2 param get /rosapi params_glob
ros2 service call /rosapi/get_param rosapi_msgs/srv/GetParam \
  "{name: '/robot_state_publisher:robot_description'}"
```

检查白名单是否允许目标参数，并按上一节解码响应的 `value`，确认XML非空且包含 `<robot`。命令行输出包含服务响应包装，不能要求整段输出直接以 `<?xml` 开头。
`/rosbridge_websocket` 未声明 `params_glob` 时，其 `param get` 失败不能证明白名单没有生效；优先检查 `/rosapi` 的实际配置与服务返回。参数列表为空也不能单独证明目标参数不可读，仍需直接调用上述服务。

URDF读取成功但模型外观缺失时，继续检查下一节的mesh地址、HTTP状态和上位机资源地址配置。

## 4. 下载 URDF 引用资源（HTTP 8090）

`GET /{package}/{relative_path}`，无请求体。`package` 是已安装 ROS 包名，
服务通过 ament 索引寻找其 share 目录。URI 重写示例：

```text
package://ur_description/meshes/ur5e/visual/base.dae
  -> http://HOST:8090/ur_description/meshes/ur5e/visual/base.dae
```

```bash
curl -I http://HOST:8090/ur_description/meshes/ur5e/visual/base.dae
```

成功 `200`，正文为资源文件原始字节，带 `Content-Length`；`HEAD` 无正文；
`OPTIONS` 为 `204`，同样支持通配 CORS。

| 扩展名 | Content-Type |
| --- | --- |
| `.dae` / `.stl` / `.obj` | `model/vnd.collada+xml` / `model/stl` / `model/obj` |
| `.png` / `.jpg`、`.jpeg` | `image/png` / `image/jpeg` |
| `.mtl` | `text/plain` |
| `.tga` / `.bmp` / `.svg` | `image/x-tga` / `image/bmp` / `image/svg+xml` |
| `.urdf` / `.xacro` / `.xml` | `application/xml`，返回原文件，不展开宏 |
| 其他 | `application/octet-stream` |

错误正文为 **纯文本，不是 JSON**：404 `mesh not found\n`（包/文件不存在或路径无效）、
405 `Only GET, HEAD and OPTIONS are supported\n`、500 `read error\n`。
路径解码一次后拒绝 `.`、`..`、反斜杠等；查询串被忽略，不提供资源列表或上传接口。
已安装包中的符号链接被视为可信资源，支持 symlink-install。
没有 Range、ETag 或应用鉴权。DAE/OBJ 引用的纹理也需要按相对资源路径加载。

## 5. 当前 WebSocket 地图业务参数与响应

以下 service 类型前缀均为 `agv_bridge_v2_interfaces/srv/`。
统一信封为 `call_service`；响应为 `service_response`，业务数据位于 `values`。

| service | 类型 | args | values 字段 |
| --- | --- | --- | --- |
| `/agv/list_maps` | `ListMaps` | `{}` | `success:bool, message:string, map_names:string[]` |
| `/agv/get_map` | `GetMap` | `{map_name:string}` | `success, message, map_name, map:nav_msgs/OccupancyGrid` |
| `/agv/start_mapping` | `StartMapping` | `{}` | `success, message` |
| `/agv/save_map` | `SaveMap` | `{map_name:string}` | `success, message, map_name` |
| `/agv/load_map` | `LoadMap` | `{map_name:string}` | `success, message, map_name` |
| `/agv/relocalize` | `Relocalize` | `{map_name:string,x:number,y:number,yaw:number}` | `success, message, map_name` |
| `/agv/set_control` | `SetControl` | `{action:"start"或"stop"或"reset"}` | `success, message, state:string` |

位置单位米、yaw 弧度，均在 map 系。`list_maps` 只列出 pgm+yaml 齐全的地图。
空地图名、后端地图源格式等处理以对应 handler 为准，客户端建议显式提供非空地图名。

```json
{"op":"call_service","id":"maps-1","service":"/agv/list_maps","args":{},"timeout":10.0}
```

```json
{"op":"service_response","id":"maps-1","service":"/agv/list_maps",
 "result":true,"values":{"success":true,"message":"示意消息","map_names":["map1022Pro"]}}
```

必须区分外层 `result`（服务调用层）和内层 `values.success`（业务层）。
建图、保存、切图、重定位涉及异步线程，成功响应可能只表示已接受，
**不能把 save_map 的 success 当作 PLY 已生成或 Nav2 已激活**。
观察 `/agv/status.mode/map_name`、核实二维档案保存结果后，再查询 8089；遇到 404 需结合保存状态重试。
`NAVIGATION` 和 `pose_initialized=true` 也不保证 Nav2 所有节点已激活。
当前 `AgvStatus.msg` 没有 `nav_ready` 字段，启动窗口内 action 仍可能以 status=6 和具体原因失败。

建议调用顺序：建图保存 → 等待业务转换和地图档案落盘 → 查询 3D status → ready 后下载。
模型显示顺序：读 URDF → package URI 重写 → HTTP 下载资源 → 用 `/joint_states` 和 `/agv/pose` 更新模型。

完整导航 action、订阅数据和迁移说明见 [rosbridge_integration.md](rosbridge_integration.md)，
机械臂参数另见 [arm_move_integration.md](arm_move_integration.md)。

## 6. 本次核验与范围

2026-10-23 本机现场：`/maps/map1022Pro/status` 返回 200 ready，字段与上表一致；
`HEAD /ur_description/meshes/ur5e/visual/base.dae` 返回 200、Collada MIME、358056 字节；
`/rosapi/get_param` 返回 JSON 编码字符串，解码后为 38011 字符完整 URDF，含 package URI。
URDF 查询实测通过 ROS service 完成；WebSocket 信封依据现有 rosbridge 契约整理。
未执行导航、建图或重定位操作；错误码以服务代码核对，未逐项在运行机器人上故障注入。
