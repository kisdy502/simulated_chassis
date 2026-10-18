# 上位机侧修复指引:URDF 读取 404(get_param 调用格式)

> 交付对象:nav-deployer 上位机开发
> 结论先行:**仿真侧 U1/U2 已全部就绪并实测通过,上位机 404 的根因是 rosapi `get_param` 的 `name` 字段格式不对——把斜杠/裸名改成冒号分隔即可,一处改动**。

---

## 1. 根因

仿真机器人环境为 ROS 2 Humble,其自带版本 rosapi 的 `GetParam.srv` 定义为:

```
string name
string default_value
---
string value
```

服务端源码(`rosapi_node`)对 `name` 的解析是**按冒号切分**:

```python
def _get_node_and_param_name(self, param):
    return tuple(param.split(":"))
```

即要求格式 **`<节点名>:<参数名>`**。传入 `robot_description`、`/robot_state_publisher/robot_description` 等斜杠或裸名形式都会被判定 malformed,**静默返回空串**(仅 rosapi 日志里有 warning),上位机读不到 → 按约定回 404。

## 2. 上位机需要改的地方

在调用 rosbridge `call_service /rosapi/get_param` 的位置(即 `/api/v1/robot/urdf` 端点的数据源,`RobotController` 相关链路),把请求改为:

```json
{
  "op": "call_service",
  "id": "p1",
  "service": "/rosapi/get_param",
  "args": {
    "name": "/robot_state_publisher:robot_description"
  }
}
```

响应 `value` 即完整 URDF XML 字符串(当前实测约 43KB)。

- 只改 `name` 的分隔符,没有其他字段/服务变更;
- 若现有代码里对空串有"回退参数化模型"逻辑,不用动——修完后不会为空;
- 建议同时把返回值做一次 `value.startsWith("<?xml")` 校验,防御性拒绝异常内容。

## 3. 修完后的验证(上位机侧)

```bash
# 1. 端点应返回 200 + XML
curl -s -o /dev/null -w '%{http_code}\n' http://<上位机>:8083/api/v1/robot/urdf

# 2. 前端 3D 页面刷新:骨架示意图消失,出现 URDF 结构;
#    mesh 经 http://<机器人IP>:8090/<包名>/<相对路径> 加载(前端已按
#    VITE_URDF_MESH_BASE 配置),机械臂显示真实外观
```

仿真侧已实测的两条底层链路(供对照,无需上位机处理):

| 链路 | 实测结果 |
|---|---|
| `/rosapi/get_param`(冒号格式) | 返回 43KB URDF,含 14 处 `package://ur_description/meshes/...` |
| mesh 静态服务 8090 | URDF 引用的 14 个 mesh(7 visual .dae + 7 collision .stl)全部 HTTP 200,MIME 正确,带 CORS 头 |

## 4. 自检命令更正(此前误判的两个坑)

之前判断"白名单未生效"的两条证据均不成立,后续自检请用下面的正确命令:

| 之前的命令 | 问题 | 正确命令 |
|---|---|---|
| `ros2 param get /rosbridge_websocket params_glob` | rosbridge 启动时读取该参数但**不声明**,`param get` 永远报 "Parameter not set",不能用它自检 | `ros2 param get /rosapi params_glob` |
| `get_param_names` 返回 0 个参数 | 该服务按"节点名:参数名"串匹配 glob,裸名 glob(`robot_description`)匹配不上任何串,返回空是**预期行为**,不代表 get_param 不通 | 见下方冒号格式 get_param |

```bash
# 机器人上执行,输出含 robot_description 才算数:
ros2 param get /rosapi params_glob
# 输出:String value is: ['robot_description', '/agv_nav_server/*', '/robot_state_publisher/*']

ros2 service call /rosapi/get_param rosapi_msgs/srv/GetParam \
  "{name: '/robot_state_publisher:robot_description'}" | head -c 120
# 输出应以 <?xml ... <robot 开头
```

白名单当前值(仿真侧已配置并随 bridge 启动生效):

```
['robot_description', '/agv_nav_server/*', '/robot_state_publisher/*']
```

同时覆盖裸名(`robot_description`)与节点前缀(`/robot_state_publisher/robot_description`)两种 glob 匹配习惯,任选其一均放行。

## 5. 其余各项状态(对照《仿真机器人侧更新指引》)

| 项 | 状态 | 说明 |
|---|---|---|
| U1 robot_description 读取 | ✅ 仿真侧就绪 | 本文档第 2 节即上位机侧的最后一处改动 |
| U2 mesh 静态服务(8090) | ✅ 就绪 | bridge 启动自动拉起;系统包 `ur_description` 的 mesh 直接命中 |
| U3 轮子 TF | ✅ 无需处理 | 仿真侧复测 `robot_state_publisher` 正常发布全部 steering/wheel frame;此前缺失为测试环境运行旧代码所致,更新代码重启后 `ros2 run tf2_tools view_frames` 应见 6 个轮组 frame |
| U4 臂控制接口 | ⏸ 维持暂缓 | `simulated_dual_arm` profile 已按契约实现 `/agv/arm_move` 与双臂关节命名;UR5e 单臂薄适配待双臂方案确定后统一处理 |

## 6. 联调环境信息

- 机器人侧 bridge 常驻运行(rosbridge `9090` / mesh `8090`),SLAM 后端 `lightning`,当前未加载地图(`pose_initialized=false` 属预期,建图或 load_map 后恢复);
- 前端环境变量:`VITE_URDF_MESH_BASE=http://<机器人IP>:8090`;
- 若修改后仍 404:先在机器人上跑第 4 节两条自检命令,把输出截图发回(大概率是调用了未重启的旧实例或 name 拼写问题)。
