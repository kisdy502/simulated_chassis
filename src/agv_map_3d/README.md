# 3D 地图显示

本功能从 Cartographer PBStream 的三维占据体素导出点云，使用优化后的子图位姿变换到 ROS `map` 坐标系（米，Z 向上）。不需要录包；显示细节受三维子图分辨率限制。二维 PGM/YAML 和 Nav2 导航流程继续保留。

## 编译与启动

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select agv_map_3d agv_bridge_v2 --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch agv_bridge_v2 agv_rosbridge.launch.py \
  maps_dir:=/absolute/path/maps pbstream_file:=/absolute/path/maps/map1008Pro/map1008Pro.pbstream
```

bridge launch 默认启动独立的 HTTP 服务（8089）。可用 `map_3d_port:=8089` 修改端口，或 `enable_map_3d:=false` 关闭。它不启动新的 bridge，不控制机器人；随所在 launch 退出。

服务每十秒扫描 PBStream，两次观察到文件大小和修改时间一致后开始导出。已有地图也会生成。请求某地图状态时可以立即触发导出；保存进行中不要请求尚未完成的地图。导出串行后台执行，超时 300 秒，不阻塞定位切换。失败的结果会记录；修复问题后删除该地图的 `.3d.json` 即可重新生成。

输出与源文件同目录，支持 `maps/name/name.pbstream` 和旧的 `maps/name.pbstream`：

```text
name.pbstream
name.pgm
name.yaml
name.ply        # 二进制 little-endian XYZ 点云，默认 0.1 m 降采样
name.3d.json    # 状态、源文件版本、点数、文件大小
```

二维 PBStream 无法恢复高度，状态会报告失败，不生成伪造的三维地图。

## 上位机

`D:\workspace\testDispatch` 的 `map-3d` 分支增加：

* 后端 `/api/v1/map-3d/{robot_map_name}/status` 和 `/cloud.ply` 代理接口。
* 默认连接 `sim_agv.ws_url` 的主机、HTTP 8089 端口。
* 可配置 `map3d.base-url`，或环境变量 `MAP3D_BASE_URL=http://robot-host:8089`。
* 地图画布右上角“三维地图”按钮。优先使用 `robot_map_name`，否则使用 `map_name`。
* 高度着色、旋转、缩放、平移及同地图机器人的实时方向箭头。关闭窗口释放 GPU 资源。

先编译并重启后端，前端 `npm run build` 后按现有方式部署。三维地图走 HTTP 文件下载；遥控和 ROS 状态继续使用原 websocket。

这是保存地图的三维查看功能。实时三维建图拼接、原始高密度点云、网格模型和前端三维编辑尚未加入。

## 独立导出与检查

```bash
ros2 run agv_map_3d pbstream_to_ply input.pbstream output.ply 0.1 0.55
curl http://localhost:8089/maps/map1008Pro/status
curl -o map1008Pro.ply http://localhost:8089/maps/map1008Pro/cloud.ply
python3 -m unittest discover -s src/agv_map_3d/tests -v
```

`BUILD_TESTING=ON` 时编译 `map3d_test_fixture`，生成带非零局部位姿、全局旋转、重叠子图和自由体素的测试 PBStream。正确导出恰好一个点 `(10, 21, 5)`，验证坐标转换、去重和自由体素过滤。

实测 `map1008Pro`：20 个三维子图、559511 个点、6714273 字节；高度约 -0.55 到 3.17 米。
